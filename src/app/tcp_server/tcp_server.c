#include "tcp_server.h"

#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/dns_sd.h>
#include <zephyr/net/net_config.h>
#include <zephyr/net/socket.h>

#include "afe_manager/afe_manager.h"

LOG_MODULE_REGISTER(tcp_server);

DNS_SD_REGISTER_TCP_SERVICE(osc_dns_sd, CONFIG_NET_HOSTNAME, "_oscilloscope._tcp", "local", NULL, TCP_SERVER_PORT);

struct adc_frame
{
    uint16_t ch1;
    uint16_t ch2;
};

#define SAMPLE_QUEUE_DEPTH 128U
K_MSGQ_DEFINE(s_sample_q, sizeof(struct adc_frame), SAMPLE_QUEUE_DEPTH, 4);

static volatile int s_client_fd = -1;

static void send_reply(int fd, const char *msg)
{
    zsock_send(fd, msg, strlen(msg), 0);
}

static void handle_command(int fd, char *line)
{
    char *tokens[6];
    int n = 0;
    char *tok = strtok(line, " \t\r\n");
    while (tok && n < 6)
    {
        tokens[n++] = tok;
        tok = strtok(NULL, " \t\r\n");
    }
    if (n == 0)
        return;

    if (strcmp(tokens[0], "afe") != 0 || n < 2)
    {
        send_reply(fd, "ERR: unknown command\n");
        return;
    }

    const char *sub = tokens[1];
    int ret = -EINVAL;

    if (strcmp(sub, "gain") == 0 && n == 4)
    {
        int ch = atoi(tokens[2]);
        float pct = strtof(tokens[3], NULL);
        if (ch == 1 || ch == 2)
        {
            ret = afe_manager_setGain((afe_manager_channel_t)ch, pct);
        }
    }
    else if (strcmp(sub, "offset") == 0 && n == 4)
    {
        int ch = atoi(tokens[2]);
        float pct = strtof(tokens[3], NULL);
        if (ch == 1 || ch == 2)
        {
            ret = afe_manager_setOffset((afe_manager_channel_t)ch, pct);
        }
    }
    else if (strcmp(sub, "atten") == 0 && n == 4)
    {
        int ch = atoi(tokens[2]);
        int val = atoi(tokens[3]);
        if ((ch == 1 || ch == 2) && (val == 1 || val == 100))
        {
            ret = afe_manager_setAttenuation((afe_manager_channel_t)ch,
                                             val == 100 ? AFE_MANAGER_ATTEN_1_TO_100 : AFE_MANAGER_ATTEN_1_TO_1);
        }
    }
    else if (strcmp(sub, "coupling") == 0 && n == 4)
    {
        int ch = atoi(tokens[2]);
        if (ch == 1 || ch == 2)
        {
            afe_manager_coupling_t c = strcmp(tokens[3], "dc") == 0 ? AFE_MANAGER_COUPLING_DC : AFE_MANAGER_COUPLING_AC;
            ret = afe_manager_setCoupling((afe_manager_channel_t)ch, c);
        }
    }
    else if (strcmp(sub, "trigger") == 0 && n == 3)
    {
        afe_manager_coupling_t c = strcmp(tokens[2], "dc") == 0 ? AFE_MANAGER_COUPLING_DC : AFE_MANAGER_COUPLING_AC;
        ret = afe_manager_setTriggerCoupling(c);
    }
    else if (strcmp(sub, "interleaved") == 0 && n == 3)
    {
        ret = afe_manager_setInterleaved(atoi(tokens[2]) != 0);
    }

    if (ret == 0)
    {
        send_reply(fd, "OK\n");
    }
    else
    {
        send_reply(fd, "ERR: invalid arguments\n");
    }
}

static void serve_client(int client_fd)
{
    char rx_buf[128];
    int rx_pos = 0;

    s_client_fd = client_fd;

    while (1)
    {
        ssize_t n = zsock_recv(client_fd, rx_buf + rx_pos, sizeof(rx_buf) - rx_pos - 1U, ZSOCK_MSG_DONTWAIT);
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

        struct adc_frame frame;
        while (k_msgq_get(&s_sample_q, &frame, K_NO_WAIT) == 0)
        {
            uint8_t pkt[6] = {
                0xADU,
                0xC1U,
                (uint8_t)(frame.ch1 >> 8),
                (uint8_t)(frame.ch1 & 0xFFU),
                (uint8_t)(frame.ch2 >> 8),
                (uint8_t)(frame.ch2 & 0xFFU),
            };
            if (zsock_send(client_fd, pkt, sizeof(pkt), 0) < 0)
            {
                goto disconnect;
            }
        }

        k_sleep(K_MSEC(1));
    }

disconnect:
    s_client_fd = -1;
    zsock_close(client_fd);
    k_msgq_purge(&s_sample_q);
    LOG_INF("Client disconnected");
}

#define SERVER_THREAD_STACK 2048U
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
        .sin_port = htons(TCP_SERVER_PORT),
        .sin_addr = {.s_addr = INADDR_ANY},
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
        socklen_t client_len = sizeof(client_addr);
        int client_fd = zsock_accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
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
    k_thread_create(&s_server_thread, s_server_stack, K_THREAD_STACK_SIZEOF(s_server_stack), server_thread_fn, NULL,
                    NULL, NULL, SERVER_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&s_server_thread, "tcp_server");
    LOG_INF("TCP server thread started");
    return 0;
}

void tcp_server_push_sample(uint16_t ch1, uint16_t ch2)
{
    if (s_client_fd < 0)
    {
        return;
    }
    struct adc_frame frame = {.ch1 = ch1, .ch2 = ch2};
    /* K_NO_WAIT: drop sample rather than block if queue is full */
    k_msgq_put(&s_sample_q, &frame, K_NO_WAIT);
}
