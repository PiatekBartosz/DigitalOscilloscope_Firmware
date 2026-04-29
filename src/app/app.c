#include "app.h"
#include "afe_manager/afe_manager.h"
#include "parallel_link/parallel_link.h"
#include "tcp_server/tcp_server.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app);

struct adc_sample
{
    uint16_t ch1;
    uint16_t ch2;
};

#define SAMPLE_QUEUE_DEPTH 64U
K_MSGQ_DEFINE(s_sample_queue, sizeof(struct adc_sample), SAMPLE_QUEUE_DEPTH, 2);

static void on_adc_sample(uint16_t ch1, uint16_t ch2)
{
    struct adc_sample s = {.ch1 = ch1, .ch2 = ch2};
    k_msgq_put(&s_sample_queue, &s, K_NO_WAIT);
}

int app_init(void)
{
    int errorCode = 0;

    do
    {
        errorCode = afe_manager_init();
        if (errorCode != 0)
        {
            LOG_ERR("Error initializing afe manager: %d", errorCode);
        }

        errorCode = tcp_server_init();
        if (errorCode != 0)
        {
            LOG_ERR("Error initializing TCP server: %d", errorCode);
        }

        errorCode = parallel_link_init(on_adc_sample);
        if (errorCode != 0)
        {
            LOG_ERR("Error initializing parallel link: %d", errorCode);
        }

        errorCode = parallel_link_write(PLINK_OP_CTRL_REG, PLINK_CTRL_CAPTURE_EN | PLINK_CTRL_MOCK_EN);
        if (errorCode != 0)
        {
            LOG_ERR("Error configuring FPGA CTRL register: %d", errorCode);
        }

        errorCode = parallel_link_stream_enable();
        if (errorCode != 0)
        {
            LOG_ERR("Error enabling parallel link streaming: %d", errorCode);
        }

        LOG_INF("App initialized — forwarding ADC samples to TCP");

        struct adc_sample s;
        while (true)
        {
            if (k_msgq_get(&s_sample_queue, &s, K_MSEC(100)) == 0)
            {
                tcp_server_push_sample(s.ch1, s.ch2);
            }
        }

    } while (0);

    return errorCode;
}
