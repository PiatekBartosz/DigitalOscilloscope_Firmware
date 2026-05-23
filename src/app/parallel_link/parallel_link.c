#include "parallel_link.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(parallel_link, CONFIG_PARALLEL_LINK_LOG_LEVEL);

#define GPIO_FROM_ALIAS(alias) GPIO_DT_SPEC_GET(DT_ALIAS(alias), gpios)

static const struct gpio_dt_spec s_ctrl[8] = {
    GPIO_FROM_ALIAS(fpga_ctrl0), GPIO_FROM_ALIAS(fpga_ctrl1),
    GPIO_FROM_ALIAS(fpga_ctrl2), GPIO_FROM_ALIAS(fpga_ctrl3),
    GPIO_FROM_ALIAS(fpga_ctrl4), GPIO_FROM_ALIAS(fpga_ctrl5),
    GPIO_FROM_ALIAS(fpga_ctrl6), GPIO_FROM_ALIAS(fpga_ctrl7),
};

static const struct gpio_dt_spec s_rw      = GPIO_FROM_ALIAS(fpga_rw);
static const struct gpio_dt_spec s_req     = GPIO_FROM_ALIAS(fpga_req);
static const struct gpio_dt_spec s_ack     = GPIO_FROM_ALIAS(fpga_ack);
static const struct gpio_dt_spec s_areset_n = GPIO_FROM_ALIAS(fpga_areset_n);

static const struct gpio_dt_spec s_data[14] = {
    GPIO_FROM_ALIAS(fpga_data0),  GPIO_FROM_ALIAS(fpga_data1),
    GPIO_FROM_ALIAS(fpga_data2),  GPIO_FROM_ALIAS(fpga_data3),
    GPIO_FROM_ALIAS(fpga_data4),  GPIO_FROM_ALIAS(fpga_data5),
    GPIO_FROM_ALIAS(fpga_data6),  GPIO_FROM_ALIAS(fpga_data7),
    GPIO_FROM_ALIAS(fpga_data8),  GPIO_FROM_ALIAS(fpga_data9),
    GPIO_FROM_ALIAS(fpga_data10), GPIO_FROM_ALIAS(fpga_data11),
    GPIO_FROM_ALIAS(fpga_data12), GPIO_FROM_ALIAS(fpga_data13),
};

#define ACK_TIMEOUT_US 500U
#define RESET_PULSE_US 10U

/*
 * Port-level bitmasks for CTRL and DATA buses.
 *
 * CTRL scatter map (device tree):
 *   GPIOB: ctrl0=PB11(b11), ctrl1=PB0(b0)
 *   GPIOD: ctrl3=PD2(b2),   ctrl4=PD15(b15)
 *   GPIOE: ctrl2=PE14(b14), ctrl5=PE9(b9), ctrl6=PE11(b11), ctrl7=PE13(b13)
 *          rw=PE8(b8)  — same port as ctrl2/5/6/7, folded into one write
 *
 * DATA scatter map (device tree):
 *   GPIOF: data0=PF1(b1),   data1=PF0(b0)
 *   GPIOE: data2=PE6(b6),   data3=PE5(b5),   data4=PE4(b4),
 *          data9=PE10(b10), data10=PE0(b0),  data11=PE15(b15),
 *          data12=PE2(b2),  data13=PE12(b12)
 *   GPIOD: data5=PD3(b3),   data6=PD4(b4),  data7=PD5(b5),  data8=PD6(b6)
 */
#define MASK_CTRL_GPIOB  ((1U << 11) | (1U << 0))
#define MASK_CTRL_GPIOD  ((1U << 15) | (1U << 2))
#define MASK_CTRL_GPIOE  ((1U << 14) | (1U << 13) | (1U << 11) | (1U << 9))
#define MASK_RW_GPIOE    (1U << 8)

static struct k_mutex s_bus_mutex;

/*
 * Write CTRL[7:0] and RW in three port-level writes.
 * ctrl2/ctrl5/ctrl6/ctrl7 and rw all live on GPIOE — merged into one write.
 * s_ctrl[0].port = GPIOB, s_ctrl[3].port = GPIOD, s_ctrl[2].port = GPIOE.
 */
static void write_ctrl_rw(uint8_t word, int rw)
{
    uint32_t pb = (((word >> 0) & 1U) << 11) |   /* ctrl0 → PB11 */
                  (((word >> 1) & 1U) << 0);      /* ctrl1 → PB0  */
    gpio_port_set_masked_raw(s_ctrl[0].port, MASK_CTRL_GPIOB, pb);

    uint32_t pd = (((word >> 3) & 1U) << 2)  |   /* ctrl3 → PD2  */
                  (((word >> 4) & 1U) << 15);     /* ctrl4 → PD15 */
    gpio_port_set_masked_raw(s_ctrl[3].port, MASK_CTRL_GPIOD, pd);

    uint32_t pe = (((word >> 2) & 1U) << 14) |   /* ctrl2 → PE14 */
                  (((word >> 5) & 1U) << 9)  |   /* ctrl5 → PE9  */
                  (((word >> 6) & 1U) << 11) |   /* ctrl6 → PE11 */
                  (((word >> 7) & 1U) << 13) |   /* ctrl7 → PE13 */
                  ((uint32_t)rw << 8);            /* rw    → PE8  */
    gpio_port_set_masked_raw(s_ctrl[2].port, MASK_CTRL_GPIOE | MASK_RW_GPIOE, pe);
}

/*
 * Read DATA[13:0] in three port-level reads.
 * s_data[0].port = GPIOF, s_data[2].port = GPIOE, s_data[5].port = GPIOD.
 */
static uint16_t read_data(void)
{
    gpio_port_value_t pf, pe, pd;
    gpio_port_get_raw(s_data[0].port, &pf);
    gpio_port_get_raw(s_data[2].port, &pe);
    gpio_port_get_raw(s_data[5].port, &pd);

    return (uint16_t)(
        (((pf >> 1)  & 1U) << 0)  |   /* data0  = PF1  */
        (((pf >> 0)  & 1U) << 1)  |   /* data1  = PF0  */
        (((pe >> 6)  & 1U) << 2)  |   /* data2  = PE6  */
        (((pe >> 5)  & 1U) << 3)  |   /* data3  = PE5  */
        (((pe >> 4)  & 1U) << 4)  |   /* data4  = PE4  */
        (((pd >> 3)  & 1U) << 5)  |   /* data5  = PD3  */
        (((pd >> 4)  & 1U) << 6)  |   /* data6  = PD4  */
        (((pd >> 5)  & 1U) << 7)  |   /* data7  = PD5  */
        (((pd >> 6)  & 1U) << 8)  |   /* data8  = PD6  */
        (((pe >> 10) & 1U) << 9)  |   /* data9  = PE10 */
        (((pe >> 0)  & 1U) << 10) |   /* data10 = PE0  */
        (((pe >> 15) & 1U) << 11) |   /* data11 = PE15 */
        (((pe >> 2)  & 1U) << 12) |   /* data12 = PE2  */
        (((pe >> 12) & 1U) << 13)     /* data13 = PE12 */
    ) & PLINK_ADC_MASK;
}

/* Execute one bus transaction. Caller must hold s_bus_mutex. */
static uint16_t transact_nolock(uint8_t rw, uint8_t op, uint8_t payload)
{
    uint16_t result = 0xFFFFU;

    uint8_t word = (uint8_t)(((payload & 0x1FU) << 3) | (op & 0x7U));
    write_ctrl_rw(word, rw);
    k_busy_wait(1);
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
    return result;
}

static uint16_t transact(uint8_t rw, uint8_t op, uint8_t payload)
{
    k_mutex_lock(&s_bus_mutex, K_FOREVER);
    uint16_t result = transact_nolock(rw, op, payload);
    k_mutex_unlock(&s_bus_mutex);
    return result;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

int parallel_link_init(void)
{
    int ret;

    k_mutex_init(&s_bus_mutex);

    if (!gpio_is_ready_dt(&s_areset_n))
    {
        LOG_ERR("FPGA ARESET_N not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&s_areset_n, GPIO_OUTPUT_ACTIVE);
    if (ret) return ret;
    gpio_pin_set_dt(&s_areset_n, 1);

    for (int i = 0; i < 8; i++)
    {
        if (!gpio_is_ready_dt(&s_ctrl[i]))
        {
            LOG_ERR("CTRL[%d] not ready", i);
            return -ENODEV;
        }
        ret = gpio_pin_configure_dt(&s_ctrl[i], GPIO_OUTPUT_INACTIVE);
        if (ret) return ret;
    }

    if (!gpio_is_ready_dt(&s_rw))
    {
        LOG_ERR("RW not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&s_rw, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    if (!gpio_is_ready_dt(&s_req))
    {
        LOG_ERR("REQ not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&s_req, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    for (int i = 0; i < 14; i++)
    {
        if (!gpio_is_ready_dt(&s_data[i]))
        {
            LOG_ERR("DATA[%d] not ready", i);
            return -ENODEV;
        }
        ret = gpio_pin_configure_dt(&s_data[i], GPIO_INPUT);
        if (ret) return ret;
    }

    if (!gpio_is_ready_dt(&s_ack))
    {
        LOG_ERR("ACK not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&s_ack, GPIO_INPUT);
    if (ret) return ret;

#ifdef CONFIG_PLINK_DEBUG
    uint16_t dev_id = transact(1, PLINK_OP_RESET, 0U);
    if (dev_id != PLINK_DEVICE_ID_EXPECTED)
    {
        LOG_ERR("FPGA device ID mismatch: got 0x%04X, expected 0x%04X",
                dev_id, PLINK_DEVICE_ID_EXPECTED);
        return -EIO;
    }
    LOG_INF("FPGA device ID OK (0x%04X)", dev_id);
#endif

    LOG_INF("Parallel link initialized");
    return 0;
}

int parallel_link_write(uint8_t op, uint8_t value)
{
    LOG_DBG("WRITE op=0x%X val=0x%02X", op, value);
    uint16_t result = transact(0, op, value);
    if (result == 0xFFFFU) {
        LOG_ERR("WRITE op=0x%X val=0x%02X: ACK timeout", op, value);
        return -EIO;
    }
    return 0;
}

uint16_t parallel_link_read(uint8_t op)
{
    return transact(1, op, 0U);
}

uint16_t parallel_link_read_status(void)
{
    return transact(1, PLINK_OP_STATUS, 0U);
}

int parallel_link_reset(void)
{
    int ret = gpio_pin_set_dt(&s_areset_n, 0);
    if (ret)
    {
        LOG_ERR("Failed to assert FPGA reset: %d", ret);
        return ret;
    }

    k_busy_wait(RESET_PULSE_US);

    ret = gpio_pin_set_dt(&s_areset_n, 1);
    if (ret)
    {
        LOG_ERR("Failed to de-assert FPGA reset: %d", ret);
        return ret;
    }

    k_busy_wait(100);
    LOG_INF("FPGA asynchronous reset pulsed");
    return 0;
}

int parallel_link_set_sample_size(uint16_t count)
{
    if (count == 0 || count > 8192U || (count & (count - 1U)) != 0U)
    {
        LOG_ERR("sample_size: %u is not a power-of-2 in [1, 8192]", count);
        return -EINVAL;
    }

    uint8_t exp = 0;
    uint16_t v = count;
    while (v > 1U) { v >>= 1; exp++; }

    LOG_DBG("sample_size: count=%u exp=%u", count, exp);
    return parallel_link_write(PLINK_OP_SAMPLE_SIZE, exp);
}

int parallel_link_acquire(uint16_t *ch1, uint16_t *ch2, uint16_t count,
                          uint32_t timeout_ms)
{
    int64_t  deadline   = k_uptime_get() + (int64_t)timeout_ms;
    uint32_t poll_count = 0;

    LOG_DBG("acquire: waiting for batch_ready (timeout %u ms)", timeout_ms);

    while (true)
    {
        uint16_t status = transact(1, PLINK_OP_STATUS, 0U);

        if (poll_count % 1000U == 0U)
        {
            LOG_DBG("acquire: poll=%u STATUS=0x%04x "
                    "(batch_rdy=%d fifo_ovf=%d sdram_bsy=%d)",
                    poll_count, status,
                    (status & PLINK_STATUS_BATCH_RDY) ? 1 : 0,
                    (status & PLINK_STATUS_FIFO_OVF)  ? 1 : 0,
                    (status & PLINK_STATUS_SDRAM_BSY) ? 1 : 0);
        }
        poll_count++;

        if (status & PLINK_STATUS_BATCH_RDY)
        {
            LOG_DBG("acquire: batch_ready after ~%u ms", poll_count);
            break;
        }

        if (k_uptime_get() >= deadline)
        {
            LOG_ERR("acquire: batch_ready timeout after %u ms "
                    "(last STATUS=0x%04x)", timeout_ms, status);
            return -ETIMEDOUT;
        }
        k_sleep(K_MSEC(1));
    }

    /* Hold mutex for the entire bulk read to eliminate per-transact lock overhead. */
    LOG_DBG("acquire: reading %u sample pairs", count);
    k_mutex_lock(&s_bus_mutex, K_FOREVER);
    for (uint16_t i = 0; i < count; i++)
    {
        ch1[i] = transact_nolock(1, PLINK_OP_CH1, 0U);
        ch2[i] = transact_nolock(1, PLINK_OP_CH2, 0U);
    }
    k_mutex_unlock(&s_bus_mutex);
    LOG_DBG("acquire: done");

    return 0;
}
