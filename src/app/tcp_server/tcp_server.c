#include "tcp_server.h"

#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/dns_sd.h>
#include <zephyr/net/net_config.h>
#include <zephyr/net/socket.h>

#include "afe_manager/afe_manager.h"
#include "app.h"

LOG_MODULE_REGISTER(tcp_server, CONFIG_TCP_SERVER_LOG_LEVEL);

DNS_SD_REGISTER_TCP_SERVICE(osc_dns_sd, CONFIG_NET_HOSTNAME,
                            "_oscilloscope._tcp", "local", NULL, TCP_SERVER_PORT);

/* -------------------------------------------------------------------------
 * Waveform frame format (binary, big-endian):
 *   [0xAD][0xC1]                          2 bytes  sync
 *   [seq3..seq0]                           4 bytes  uint32 sequence number
 *   [cnt1][cnt0]                           2 bytes  uint16 sample count N
 *   [N × (ch1_hi ch1_lo ch2_hi ch2_lo)]   N*4 bytes samples
 * Total: 8 + WAVEFORM_SAMPLES * 4 bytes
 * ------------------------------------------------------------------------- */

#define FRAME_HDR_LEN   8U
#define FRAME_SMPL_LEN  (WAVEFORM_SAMPLES * 4U)
#define FRAME_TOTAL_LEN (FRAME_HDR_LEN + FRAME_SMPL_LEN)

/* Static buffers — too large for the stack. */
static uint16_t s_ch1[WAVEFORM_SAMPLES];
static uint16_t s_ch2[WAVEFORM_SAMPLES];
static uint8_t  s_tx_buf[FRAME_TOTAL_LEN];
static uint32_t s_wf_seq = 0;

/* Count of samples stored by the last raw_write command.
 * raw_read uses this so it sends exactly as many samples as were captured,
 * even if sample_size has been changed since. */
static uint16_t s_raw_count = 0;

/* -------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */

static void send_reply(int fd, const char *msg)
{
    zsock_send(fd, msg, strlen(msg), 0);
}

static bool send_waveform(int fd, uint16_t count)
{
    uint32_t frame_len = FRAME_HDR_LEN + (uint32_t)count * 4U;

    s_tx_buf[0] = 0xADU;
    s_tx_buf[1] = 0xC1U;
    s_tx_buf[2] = (uint8_t)(s_wf_seq >> 24);
    s_tx_buf[3] = (uint8_t)(s_wf_seq >> 16);
    s_tx_buf[4] = (uint8_t)(s_wf_seq >> 8);
    s_tx_buf[5] = (uint8_t)(s_wf_seq);
    s_tx_buf[6] = (uint8_t)(count >> 8);
    s_tx_buf[7] = (uint8_t)(count);

    for (uint16_t i = 0; i < count; i++)
    {
        uint8_t *p = s_tx_buf + FRAME_HDR_LEN + i * 4U;
        p[0]       = (uint8_t)(s_ch1[i] >> 8);
        p[1]       = (uint8_t)(s_ch1[i]);
        p[2]       = (uint8_t)(s_ch2[i] >> 8);
        p[3]       = (uint8_t)(s_ch2[i]);
    }

    s_wf_seq++;

    /* TCP may accept fewer bytes than requested — loop until all bytes are sent. */
    size_t sent = 0;
    while (sent < frame_len)
    {
        ssize_t n = zsock_send(fd, s_tx_buf + sent, frame_len - sent, 0);
        if (n <= 0)
        {
            LOG_ERR("send_waveform: send error at offset %zu (ret=%d errno=%d)",
                    sent, (int)n, errno);
            return false;
        }
        sent += (size_t)n;
    }
    LOG_DBG("send_waveform OK seq=%u count=%u (%u bytes)", s_wf_seq - 1, count, frame_len);
    return true;
}

/* -------------------------------------------------------------------------
 * Command handler
 * ------------------------------------------------------------------------- */

static void handle_command(int fd, char *line)
{
    LOG_DBG("CMD: %s", line);

    char *tokens[6];
    int   n   = 0;
    char *tok = strtok(line, " \t\r\n");
    while (tok && n < 6)
    {
        tokens[n++] = tok;
        tok         = strtok(NULL, " \t\r\n");
    }
    if (n == 0)
        return;

    /* "acquire" — capture one batch and send it as a binary frame */
    //TODO: change for strcmpn
    if (strcmp(tokens[0], "acquire") == 0)
    {
        uint16_t count = app_get_sample_size();
        int ret = app_acquire(s_ch1, s_ch2);
        if (ret != 0)
        {
            LOG_ERR("acquire failed: %d", ret);
            send_reply(fd, "ERR: acquire failed\n");
            return;
        }
        if (!send_waveform(fd, count))
        {
            LOG_ERR("send_waveform failed (seq=%u)", s_wf_seq - 1);
        }
        return;
    }

    /* raw_write — capture into firmware buffers without sending to client.
     * Reply: "OK <count>\n".  Client can then call raw_read one or more times
     * to retrieve the same snapshot. */
    if (strcmp(tokens[0], "raw_write") == 0)
    {
        s_raw_count = app_get_sample_size();
        int ret = app_acquire(s_ch1, s_ch2);
        if (ret != 0)
        {
            s_raw_count = 0;
            LOG_ERR("raw_write: acquire failed: %d", ret);
            send_reply(fd, "ERR: raw_write failed\n");
            return;
        }
        char reply[32];
        snprintf(reply, sizeof(reply), "OK %u\n", s_raw_count);
        send_reply(fd, reply);
        return;
    }

    /* raw_read — send the buffers stored by the last raw_write as a binary
     * waveform frame (same format as acquire).  Idempotent: may be called
     * multiple times to verify the same snapshot is returned each time.
     * Prefixed with a "RAW_READ <count>\n" text line so the command panel
     * logs it as well as the oscilloscope plot updating. */
    if (strcmp(tokens[0], "raw_read") == 0)
    {
        uint16_t count = s_raw_count ? s_raw_count : app_get_sample_size();
        char hdr[32];
        snprintf(hdr, sizeof(hdr), "RAW_READ %u\n", count);
        send_reply(fd, hdr);
        if (!send_waveform(fd, count))
        {
            LOG_ERR("raw_read: send_waveform failed");
        }
        return;
    }

    /* raw_reset_write_ptr — pulse RESET_FIFO so the FPGA sample_buffer
     * returns to ST_FILLING at address 0.  Use this to start a fresh capture
     * if the previous write was aborted or the FPGA got stuck. */
    if (strcmp(tokens[0], "raw_reset_write_ptr") == 0)
    {
        int ret = app_reset_fpga_buffer();
        send_reply(fd, ret == 0 ? "OK\n" : "ERR: reset failed\n");
        return;
    }

    /* raw_write_read — atomic capture + immediate send.
     * Convenience shortcut for raw_write followed by raw_read.
     * Stores the snapshot in s_ch1/s_ch2 so a subsequent raw_read still
     * returns the same data. */
    if (strcmp(tokens[0], "raw_write_read") == 0)
    {
        s_raw_count = app_get_sample_size();
        int ret = app_acquire(s_ch1, s_ch2);
        if (ret != 0)
        {
            s_raw_count = 0;
            LOG_ERR("raw_write_read: acquire failed: %d", ret);
            send_reply(fd, "ERR: raw_write_read failed\n");
            return;
        }
        char hdr[32];
        snprintf(hdr, sizeof(hdr), "RAW_READ %u\n", s_raw_count);
        send_reply(fd, hdr);
        if (!send_waveform(fd, s_raw_count))
        {
            LOG_ERR("raw_write_read: send_waveform failed");
        }
        return;
    }

    /* "afe" sub-commands */
    if (strcmp(tokens[0], "afe") != 0 || n < 2)
    {
        send_reply(fd, "ERR: unknown command\n");
        return;
    }

    const char *sub = tokens[1];
    int         ret = -EINVAL;

    if (strcmp(sub, "gain") == 0 && n == 4)
    {
        int   ch  = atoi(tokens[2]);
        float pct = strtof(tokens[3], NULL);
        if (ch == 1 || ch == 2)
        {
            ret = afe_manager_setGain((afe_manager_channel_t)ch, pct);
        }
    }
    else if (strcmp(sub, "offset") == 0 && n == 4)
    {
        int   ch  = atoi(tokens[2]);
        float pct = strtof(tokens[3], NULL);
        if (ch == 1 || ch == 2)
        {
            ret = afe_manager_setOffset((afe_manager_channel_t)ch, pct);
        }
    }
    else if (strcmp(sub, "atten") == 0 && n == 4)
    {
        int ch  = atoi(tokens[2]);
        int val = atoi(tokens[3]);
        if ((ch == 1 || ch == 2) && (val == 1 || val == 100))
        {
            ret = afe_manager_setAttenuation(
                (afe_manager_channel_t)ch,
                val == 100 ? AFE_MANAGER_ATTEN_1_TO_100 : AFE_MANAGER_ATTEN_1_TO_1);
        }
    }
    else if (strcmp(sub, "coupling") == 0 && n == 4)
    {
        int ch = atoi(tokens[2]);
        if (ch == 1 || ch == 2)
        {
            afe_manager_coupling_t c =
                strcmp(tokens[3], "dc") == 0 ? AFE_MANAGER_COUPLING_DC
                                              : AFE_MANAGER_COUPLING_AC;
            ret = afe_manager_setCoupling((afe_manager_channel_t)ch, c);
        }
    }
    else if (strcmp(sub, "trigger") == 0 && n == 3)
    {
        afe_manager_coupling_t c =
            strcmp(tokens[2], "dc") == 0 ? AFE_MANAGER_COUPLING_DC
                                         : AFE_MANAGER_COUPLING_AC;
        ret = afe_manager_setTriggerCoupling(c);
    }
    else if (strcmp(sub, "interleaved") == 0 && n == 3)
    {
        ret = afe_manager_setInterleaved(atoi(tokens[2]) != 0);
    }
    else if (strcmp(sub, "mock_adc") == 0 && n == 3)
    {
        int val = atoi(tokens[2]);
        if (val == 0 || val == 1)
        {
            ret = app_set_mock_adc(val != 0);
        }
    }
    else if (strcmp(sub, "sample_size") == 0 && n == 3)
    {
        int val = atoi(tokens[2]);
        if (val > 0 && val <= 8192)
        {
            ret = app_set_sample_size((uint16_t)val);
        }
    }
    else if (strcmp(sub, "decim") == 0 && n == 3)
    {
        int val = atoi(tokens[2]);
        if (val >= 1 && val <= 2047)
        {
            ret = app_set_decim_factor((uint16_t)val);
        }
    }

    send_reply(fd, ret == 0 ? "OK\n" : "ERR: invalid arguments\n");
}

/* -------------------------------------------------------------------------
 * Client session
 * ------------------------------------------------------------------------- */

static void serve_client(int client_fd)
{
    char rx_buf[128];
    int  rx_pos = 0;

    while (1)
    {
        ssize_t n = zsock_recv(client_fd, rx_buf + rx_pos,
                               sizeof(rx_buf) - rx_pos - 1U, ZSOCK_MSG_DONTWAIT);
        if (n > 0)
        {
            rx_pos += (int)n;
            rx_buf[rx_pos] = '\0';

            char *nl;
            while ((nl = strchr(rx_buf, '\n')) != NULL)
            {
                *nl = '\0';
                handle_command(client_fd, rx_buf);
                int remaining = (int)(rx_pos - (nl - rx_buf) - 1);
                if (remaining > 0)
                {
                    memmove(rx_buf, nl + 1, (size_t)remaining);
                }
                rx_pos = remaining > 0 ? remaining : 0;
            }
            if (rx_pos >= (int)(sizeof(rx_buf) - 1))
            {
                rx_pos = 0;
            }
        }
        else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
        {
            break;
        }

        k_sleep(K_MSEC(1));
    }

    zsock_close(client_fd);
    LOG_INF("Client disconnected");
}

/* -------------------------------------------------------------------------
 * Server thread
 * ------------------------------------------------------------------------- */

#define SERVER_THREAD_STACK 4096U
#define SERVER_THREAD_PRIO  7
static K_THREAD_STACK_DEFINE(s_server_stack, SERVER_THREAD_STACK);
static struct k_thread s_server_thread;

static void server_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    (void)net_config_init_app(NULL, "TCP server waiting for network");

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(TCP_SERVER_PORT),
        .sin_addr   = {.s_addr = INADDR_ANY},
    };

    int server_fd = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_fd < 0)
    {
        LOG_ERR("socket() failed: %d", errno);
        return;
    }

    int opt = 1;
    zsock_setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (zsock_bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        LOG_ERR("bind() failed: %d", errno);
        zsock_close(server_fd);
        return;
    }

    if (zsock_listen(server_fd, 1) < 0)
    {
        LOG_ERR("listen() failed: %d", errno);
        zsock_close(server_fd);
        return;
    }

    LOG_INF("TCP server listening on port %d", TCP_SERVER_PORT);

    while (1)
    {
        struct sockaddr_in client_addr;
        socklen_t          client_len = sizeof(client_addr);
        int client_fd = zsock_accept(server_fd, (struct sockaddr *)&client_addr,
                                     &client_len);
        if (client_fd < 0)
        {
            LOG_ERR("accept() failed: %d", errno);
            k_sleep(K_MSEC(100));
            continue;
        }
        LOG_INF("Client connected");
        serve_client(client_fd);
    }
}

int tcp_server_init(void)
{
    k_thread_create(&s_server_thread, s_server_stack,
                    K_THREAD_STACK_SIZEOF(s_server_stack),
                    server_thread_fn, NULL, NULL, NULL,
                    SERVER_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&s_server_thread, "tcp_server");
    LOG_INF("TCP server thread started");
    return 0;
}
