#include "app.h"
#include "afe_manager/afe_manager.h"
#include "parallel_link/parallel_link.h"
#include "tcp_server/tcp_server.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app, CONFIG_APP_LOG_LEVEL);

#define APP_DEFAULT_FPGA_CTRL   PLINK_CTRL_CAPTURE_EN /* capture on, trigger/mock off, decim=1 */
#define APP_DEFAULT_SAMPLE_SIZE 8192U
#define APP_DEFAULT_PRETRIGGER  0U

/* Shadow of the full 14-bit FPGA CTRL register. */
static uint16_t s_fpga_ctrl = APP_DEFAULT_FPGA_CTRL;

/* Current capture depth, mirroring the FPGA OP_SAMPLE_SIZE register. */
static uint16_t s_sample_size = APP_DEFAULT_SAMPLE_SIZE;

/* Current pretrigger depth in samples; 0 = disabled. */
static uint16_t s_pretrigger_count = APP_DEFAULT_PRETRIGGER;

/* Timeout for waiting on the FPGA sample buffer to fill.
 * At 1 kHz mock rate with 8192 samples this is ~8.2 s; allow generous margin. */
#define ACQUIRE_TIMEOUT_MS 30000U

int app_set_sample_size(uint16_t count)
{
    if (s_pretrigger_count != 0U && s_pretrigger_count >= count)
    {
        LOG_ERR("Sample size %u must exceed configured pretrigger %u", count, s_pretrigger_count);
        return -EINVAL;
    }

    int ret = parallel_link_set_sample_size(count);
    if (ret == 0)
    {
        s_sample_size = count;
        LOG_INF("Sample size set to %u", count);
    }

    return ret;
}

uint16_t app_get_sample_size(void)
{
    return s_sample_size;
}

int app_set_pretrigger(uint16_t count)
{
    if (count > PLINK_PRETRIGGER_MAX || (count != 0U && count >= s_sample_size))
    {
        LOG_ERR("Pretrigger %u must be in [0, %u] and smaller than sample size %u", count, PLINK_PRETRIGGER_MAX,
                s_sample_size);
        return -EINVAL;
    }
    int ret = parallel_link_set_pretrigger(count);
    if (ret == 0)
    {
        s_pretrigger_count = count;
        LOG_INF("Pretrigger set to %u samples", count);
    }
    return ret;
}

uint16_t app_get_pretrigger(void)
{
    return s_pretrigger_count;
}

int app_set_mock_adc(bool enable)
{
    if (enable)
    {
        s_fpga_ctrl |= PLINK_CTRL_MOCK_EN;
    }
    else
    {
        s_fpga_ctrl &= ~(uint16_t)PLINK_CTRL_MOCK_EN;
    }
    return parallel_link_write(PLINK_ADDR_CTRL, s_fpga_ctrl);
}

int app_set_decim_factor(uint16_t factor)
{
    if (factor < 1U || factor > PLINK_CTRL_DECIM_MAX)
    {
        LOG_ERR("decim_factor %u out of range [1, %u]", factor, PLINK_CTRL_DECIM_MAX);
        return -EINVAL;
    }
    s_fpga_ctrl = (s_fpga_ctrl & 0x000FU) | PLINK_CTRL_DECIM(factor);
    LOG_INF("Decimation factor set to %u (ctrl=0x%04X)", factor, s_fpga_ctrl);
    return parallel_link_write(PLINK_ADDR_CTRL, s_fpga_ctrl);
}

uint16_t app_get_decim_factor(void)
{
    uint16_t factor = (s_fpga_ctrl >> PLINK_CTRL_DECIM_SHIFT) & PLINK_CTRL_DECIM_MAX;
    return factor == 0U ? 1U : factor;
}

int app_set_trigger_mode(bool enable)
{
    if (enable)
    {
        s_fpga_ctrl |= PLINK_CTRL_TRIGGER_EN;
    }
    else
    {
        s_fpga_ctrl &= ~(uint16_t)PLINK_CTRL_TRIGGER_EN;
    }
    LOG_INF("Trigger mode %s (ctrl=0x%04X)", enable ? "normal" : "off", s_fpga_ctrl);
    return parallel_link_write(PLINK_ADDR_CTRL, s_fpga_ctrl);
}

bool app_get_trigger_mode(void)
{
    return (s_fpga_ctrl & PLINK_CTRL_TRIGGER_EN) != 0U;
}

bool app_get_mock_adc(void)
{
    return (s_fpga_ctrl & PLINK_CTRL_MOCK_EN) != 0U;
}

uint8_t app_get_fpga_status(void)
{
    return parallel_link_read_status();
}

int app_reset_fpga_buffer(void)
{
    int ret = parallel_link_write(PLINK_ADDR_CTRL, s_fpga_ctrl | (uint16_t)PLINK_CTRL_RESET_FIFO);
    if (ret != 0)
    {
        LOG_ERR("Failed to assert RESET_FIFO: %d", ret);
        return ret;
    }
    ret = parallel_link_write(PLINK_ADDR_CTRL, s_fpga_ctrl);
    if (ret != 0)
    {
        LOG_ERR("Failed to de-assert RESET_FIFO: %d", ret);
    }
    return ret;
}

int app_acquire(uint16_t *ch1, uint16_t *ch2)
{
    LOG_DBG("app_acquire: ctrl_shadow=0x%02x (cap=%d mock=%d) count=%u", s_fpga_ctrl,
            (s_fpga_ctrl & PLINK_CTRL_CAPTURE_EN) ? 1 : 0, (s_fpga_ctrl & PLINK_CTRL_MOCK_EN) ? 1 : 0, s_sample_size);

    int ret = app_reset_fpga_buffer();
    if (ret != 0)
    {
        return ret;
    }

    LOG_DBG("app_acquire: FPGA reset done, waiting for samples");
    return parallel_link_acquire(ch1, ch2, s_sample_size, ACQUIRE_TIMEOUT_MS);
}

int app_init(void)
{
    int ret;

    ret = afe_manager_init();
    if (ret != 0)
    {
        LOG_ERR("afe_manager_init failed: %d", ret);
        return ret;
    }

    ret = tcp_server_init();
    if (ret != 0)
    {
        LOG_ERR("tcp_server_init failed: %d", ret);
        return ret;
    }

    ret = parallel_link_init();
    if (ret != 0)
    {
        LOG_ERR("parallel_link_init failed: %d", ret);
        return ret;
    }

    ret = parallel_link_write(PLINK_ADDR_CTRL, s_fpga_ctrl);
    if (ret != 0)
    {
        LOG_ERR("Failed to write FPGA CTRL register: %d", ret);
        return ret;
    }

    LOG_INF("App initialized — waiting for acquire commands from TCP client");

    /* TCP server runs in its own thread; main thread has nothing to do. */
    while (true)
    {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
