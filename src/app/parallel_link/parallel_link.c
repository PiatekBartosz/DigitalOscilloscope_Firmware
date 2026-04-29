#include "parallel_link.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(parallel_link);

#define GPIO_FROM_ALIAS(alias) GPIO_DT_SPEC_GET(DT_ALIAS(alias), gpios)

static const struct gpio_dt_spec s_ctrl[8] = {
    GPIO_FROM_ALIAS(fpga_ctrl0), GPIO_FROM_ALIAS(fpga_ctrl1), GPIO_FROM_ALIAS(fpga_ctrl2), GPIO_FROM_ALIAS(fpga_ctrl3),
    GPIO_FROM_ALIAS(fpga_ctrl4), GPIO_FROM_ALIAS(fpga_ctrl5), GPIO_FROM_ALIAS(fpga_ctrl6), GPIO_FROM_ALIAS(fpga_ctrl7),
};

static const struct gpio_dt_spec s_rw = GPIO_FROM_ALIAS(fpga_rw);
static const struct gpio_dt_spec s_req = GPIO_FROM_ALIAS(fpga_req);

static const struct gpio_dt_spec s_data[14] = {
    GPIO_FROM_ALIAS(fpga_data0),  GPIO_FROM_ALIAS(fpga_data1),  GPIO_FROM_ALIAS(fpga_data2),
    GPIO_FROM_ALIAS(fpga_data3),  GPIO_FROM_ALIAS(fpga_data4),  GPIO_FROM_ALIAS(fpga_data5),
    GPIO_FROM_ALIAS(fpga_data6),  GPIO_FROM_ALIAS(fpga_data7),  GPIO_FROM_ALIAS(fpga_data8),
    GPIO_FROM_ALIAS(fpga_data9),  GPIO_FROM_ALIAS(fpga_data10), GPIO_FROM_ALIAS(fpga_data11),
    GPIO_FROM_ALIAS(fpga_data12), GPIO_FROM_ALIAS(fpga_data13),
};

static const struct gpio_dt_spec s_ack = GPIO_FROM_ALIAS(fpga_ack);
static const struct gpio_dt_spec s_irq = GPIO_FROM_ALIAS(fpga_irq);

#define ACK_TIMEOUT_US 500U

static struct k_mutex s_bus_mutex;
static struct k_sem s_irq_sem;

#define STREAM_THREAD_STACK 2048U
#define STREAM_THREAD_PRIO  5
static K_THREAD_STACK_DEFINE(s_stream_stack, STREAM_THREAD_STACK);
static struct k_thread s_stream_thread;

static plink_irq_cb_t s_user_cb;
static struct gpio_callback s_irq_cb_data;

static void write_ctrl(uint8_t op, uint8_t payload)
{
    /* CTRL[2:0] = op, CTRL[7:3] = payload[4:0] */
    uint8_t word = (uint8_t)(((payload & 0x1FU) << 3) | (op & 0x7U));
    for (int i = 0; i < 8; i++)
    {
        gpio_pin_set_dt(&s_ctrl[i], (word >> i) & 1U);
    }
}

static uint16_t read_data(void)
{
    uint16_t val = 0;
    for (int i = 0; i < 14; i++)
    {
        if (gpio_pin_get_dt(&s_data[i]) > 0)
        {
            val |= (uint16_t)(1U << i);
        }
    }
    return val & PLINK_ADC_MASK;
}

/* Execute one transaction.
 *
 * rw      : 0 = write to FPGA, 1 = read from FPGA
 * op      : PLINK_OP_* target
 * payload : 5-bit value to write, PAYLOAD[4:0] (ignored when rw=1)
 *
 * Returns DATA result on reads, 0 on writes, 0xFFFF on ACK timeout. */
static uint16_t transact(uint8_t rw, uint8_t op, uint8_t payload)
{
    uint16_t result = 0xFFFFU;

    k_mutex_lock(&s_bus_mutex, K_FOREVER);

    write_ctrl(op, rw ? 0U : payload);
    gpio_pin_set_dt(&s_rw, rw);
    gpio_pin_set_dt(&s_req, 1);

    uint32_t elapsed = 0;
    while (gpio_pin_get_dt(&s_ack) == 0)
    {
        k_busy_wait(1);
        if (++elapsed >= ACK_TIMEOUT_US)
        {
            LOG_ERR("ACK timeout (op=0x%X rw=%u)", op, rw);
            goto done;
        }
    }

    result = rw ? read_data() : 0U;

done:
    gpio_pin_set_dt(&s_req, 0);
    k_mutex_unlock(&s_bus_mutex);
    return result;
}

static void irq_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    k_sem_give(&s_irq_sem);
}

static void stream_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    while (1)
    {
        k_sem_take(&s_irq_sem, K_FOREVER);
        if (!s_user_cb)
        {
            continue;
        }
        uint16_t ch1 = transact(1, PLINK_OP_CH1, 0U);
        uint16_t ch2 = transact(1, PLINK_OP_CH2, 0U);
        s_user_cb(ch1, ch2);
    }
}

int parallel_link_init(plink_irq_cb_t cb)
{
    int ret;

    k_mutex_init(&s_bus_mutex);
    k_sem_init(&s_irq_sem, 0, K_SEM_MAX_LIMIT);
    s_user_cb = cb;

    for (int i = 0; i < 8; i++)
    {
        if (!gpio_is_ready_dt(&s_ctrl[i]))
        {
            LOG_ERR("CTRL[%d] not ready", i);
            return -ENODEV;
        }
        ret = gpio_pin_configure_dt(&s_ctrl[i], GPIO_OUTPUT_INACTIVE);
        if (ret)
            return ret;
    }

    if (!gpio_is_ready_dt(&s_rw))
    {
        LOG_ERR("RW not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&s_rw, GPIO_OUTPUT_INACTIVE);
    if (ret)
        return ret;

    if (!gpio_is_ready_dt(&s_req))
    {
        LOG_ERR("REQ not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&s_req, GPIO_OUTPUT_INACTIVE);
    if (ret)
        return ret;

    for (int i = 0; i < 14; i++)
    {
        if (!gpio_is_ready_dt(&s_data[i]))
        {
            LOG_ERR("DATA[%d] not ready", i);
            return -ENODEV;
        }
        ret = gpio_pin_configure_dt(&s_data[i], GPIO_INPUT);
        if (ret)
            return ret;
    }

    if (!gpio_is_ready_dt(&s_ack))
    {
        LOG_ERR("ACK not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&s_ack, GPIO_INPUT);
    if (ret)
        return ret;

    if (!gpio_is_ready_dt(&s_irq))
    {
        LOG_ERR("IRQ not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&s_irq, GPIO_INPUT);
    if (ret)
        return ret;

    if (cb)
    {
        ret = gpio_pin_interrupt_configure_dt(&s_irq, GPIO_INT_EDGE_RISING);
        if (ret)
            return ret;
        gpio_init_callback(&s_irq_cb_data, irq_isr, BIT(s_irq.pin));
        ret = gpio_add_callback(s_irq.port, &s_irq_cb_data);
        if (ret)
            return ret;

        k_thread_create(&s_stream_thread, s_stream_stack, K_THREAD_STACK_SIZEOF(s_stream_stack), stream_thread_fn, NULL,
                        NULL, NULL, STREAM_THREAD_PRIO, 0, K_NO_WAIT);
        k_thread_name_set(&s_stream_thread, "plink_stream");
    }

#ifdef CONFIG_PLINK_DEBUG
    uint16_t dev_id = transact(1, PLINK_OP_RESET, 0U);
    if (dev_id != PLINK_DEVICE_ID_EXPECTED)
    {
        LOG_ERR("FPGA device ID mismatch: got 0x%04X, expected 0x%04X", dev_id, PLINK_DEVICE_ID_EXPECTED);
        return -EIO;
    }
    LOG_INF("FPGA device ID OK (0x%04X)", dev_id);
#endif

    LOG_INF("Parallel link initialized");
    return 0;
}

uint16_t parallel_link_read_ch1(void)
{
    return transact(1, PLINK_OP_CH1, 0U);
}
uint16_t parallel_link_read_ch2(void)
{
    return transact(1, PLINK_OP_CH2, 0U);
}
uint16_t parallel_link_read_status(void)
{
    return transact(1, PLINK_OP_STATUS, 0U);
}

int parallel_link_write(uint8_t op, uint8_t value)
{
    return (transact(0, op, value) == 0xFFFFU) ? -EIO : 0;
}

uint16_t parallel_link_read(uint8_t op)
{
    return transact(1, op, 0U);
}

int parallel_link_stream_enable(void)
{
    return (transact(0, PLINK_OP_STREAM, 1U) == 0xFFFFU) ? -EIO : 0;
}

int parallel_link_stream_disable(void)
{
    return (transact(0, PLINK_OP_STREAM, 0U) == 0xFFFFU) ? -EIO : 0;
}

int parallel_link_reset(void)
{
    return (transact(0, PLINK_OP_RESET, 0U) == 0xFFFFU) ? -EIO : 0;
}
