#include "parallel_link.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(parallel_link, CONFIG_PARALLEL_LINK_LOG_LEVEL);

#define GPIO_FROM_ALIAS(alias) GPIO_DT_SPEC_GET(DT_ALIAS(alias), gpios)

/*
 * 3 address pins: CFG_ADDR[0..2].
 * Physical mapping:
 *   ctrl0 = CFG_ADDR[0] → PB11
 *   ctrl1 = CFG_ADDR[1] → PB0
 *   ctrl2 = CFG_ADDR[2] → PE14
 */
static const struct gpio_dt_spec s_ctrl[3] = {
    GPIO_FROM_ALIAS(fpga_ctrl0),
    GPIO_FROM_ALIAS(fpga_ctrl1),
    GPIO_FROM_ALIAS(fpga_ctrl2),
};

static const struct gpio_dt_spec s_rw       = GPIO_FROM_ALIAS(fpga_rw);       /* C7_Write_To_FPGA → PE8 */
static const struct gpio_dt_spec s_req      = GPIO_FROM_ALIAS(fpga_req);      /* C3_Trigger_Clock_INP   */
static const struct gpio_dt_spec s_inc      = GPIO_FROM_ALIAS(fpga_inc);      /* Increment pin          */
static const struct gpio_dt_spec s_busy     = GPIO_FROM_ALIAS(fpga_busy);     /* Status_FPGA_Busy       */
static const struct gpio_dt_spec s_areset_n = GPIO_FROM_ALIAS(fpga_areset_n);

/*
 * 14 data pins: io_mcu_data[13:0] — bidirectional PE_dataBus.
 *
 * Physical scatter map (device tree):
 *   GPIOE: data0=PE8(b8),  data1=PE7(b7),  data2=PE6(b6),  data3=PE5(b5),  data4=PE4(b4),
 *          data9=PE10(b10), data10=PE0(b0), data11=PE15(b15), data12=PE2(b2), data13=PE12(b12)
 *   GPIOF: data5=PF1(b1),  data6=PF0(b0)
 *   GPIOD: data7=PD3(b3),  data8=PD4(b4)
 *   GPIOD: rw=PD5(b5),     req=PD6(b6)   [control, same GPIOD port]
 */
static const struct gpio_dt_spec s_data[14] = {
    GPIO_FROM_ALIAS(fpga_data0),  GPIO_FROM_ALIAS(fpga_data1),
    GPIO_FROM_ALIAS(fpga_data2),  GPIO_FROM_ALIAS(fpga_data3),
    GPIO_FROM_ALIAS(fpga_data4),  GPIO_FROM_ALIAS(fpga_data5),
    GPIO_FROM_ALIAS(fpga_data6),  GPIO_FROM_ALIAS(fpga_data7),
    GPIO_FROM_ALIAS(fpga_data8),  GPIO_FROM_ALIAS(fpga_data9),
    GPIO_FROM_ALIAS(fpga_data10), GPIO_FROM_ALIAS(fpga_data11),
    GPIO_FROM_ALIAS(fpga_data12), GPIO_FROM_ALIAS(fpga_data13),
};

#define BUSY_TIMEOUT_US  500U
#define RESET_PULSE_US   10U

/* CFG_ADDR scatter masks — rw/req use gpio_pin_set_dt */
#define MASK_CTRL_GPIOB  ((1U << 11) | (1U << 0))  /* addr0=PB11, addr1=PB0  */
#define MASK_CTRL_GPIOE   (1U << 14)                /* addr2=PE14             */

/* Data bus scatter masks */
#define MASK_DATA_GPIOE  ((1U << 8) | (1U << 7) | (1U << 6) | (1U << 5) | (1U << 4) | \
                          (1U << 10) | (1U << 0) | (1U << 15) | (1U << 2) | (1U << 12))
#define MASK_DATA_GPIOF  ((1U << 1) | (1U << 0))
#define MASK_DATA_GPIOD  ((1U << 3) | (1U << 4))

static struct k_mutex s_bus_mutex;

/* Set CFG_ADDR[2:0] and rw. */
static void write_addr_rw(uint8_t addr, int rw)
{
    uint32_t pb = (((addr >> 0) & 1U) << 11) |
                  (((addr >> 1) & 1U) << 0);
    gpio_port_set_masked_raw(s_ctrl[0].port, MASK_CTRL_GPIOB, pb);

    uint32_t pe = (((addr >> 2) & 1U) << 14);
    gpio_port_set_masked_raw(s_ctrl[2].port, MASK_CTRL_GPIOE, pe);

    gpio_pin_set_dt(&s_rw, rw);
}

/*
 * Drive all 14 data pins with value (MCU → FPGA, write direction).
 * s_data[0].port = GPIOE, s_data[5].port = GPIOF, s_data[7].port = GPIOD.
 */
static void write_data_bus(uint16_t value)
{
    uint32_t pe = (((value >>  0) & 1U) << 8)  |
                  (((value >>  1) & 1U) << 7)  |
                  (((value >>  2) & 1U) << 6)  |
                  (((value >>  3) & 1U) << 5)  |
                  (((value >>  4) & 1U) << 4)  |
                  (((value >>  9) & 1U) << 10) |
                  (((value >> 10) & 1U) << 0)  |
                  (((value >> 11) & 1U) << 15) |
                  (((value >> 12) & 1U) << 2)  |
                  (((value >> 13) & 1U) << 12);
    gpio_port_set_masked_raw(s_data[0].port, MASK_DATA_GPIOE, pe);

    uint32_t pf = (((value >> 5) & 1U) << 1) |
                  (((value >> 6) & 1U) << 0);
    gpio_port_set_masked_raw(s_data[5].port, MASK_DATA_GPIOF, pf);

    uint32_t pd = (((value >> 7) & 1U) << 3) |
                  (((value >> 8) & 1U) << 4);
    gpio_port_set_masked_raw(s_data[7].port, MASK_DATA_GPIOD, pd);
}

/*
 * Sample all 14 data pins (FPGA → MCU, read direction).
 */
static uint16_t read_data_bus(void)
{
    gpio_port_value_t pe, pf, pd;
    gpio_port_get_raw(s_data[0].port, &pe);
    gpio_port_get_raw(s_data[5].port, &pf);
    gpio_port_get_raw(s_data[7].port, &pd);

    return (uint16_t)(
        (((pe >>  8) & 1U) <<  0) |   /* data0  = PE8  */
        (((pe >>  7) & 1U) <<  1) |   /* data1  = PE7  */
        (((pe >>  6) & 1U) <<  2) |   /* data2  = PE6  */
        (((pe >>  5) & 1U) <<  3) |   /* data3  = PE5  */
        (((pe >>  4) & 1U) <<  4) |   /* data4  = PE4  */
        (((pf >>  1) & 1U) <<  5) |   /* data5  = PF1  */
        (((pf >>  0) & 1U) <<  6) |   /* data6  = PF0  */
        (((pd >>  3) & 1U) <<  7) |   /* data7  = PD3  */
        (((pd >>  4) & 1U) <<  8) |   /* data8  = PD4  */
        (((pe >> 10) & 1U) <<  9) |   /* data9  = PE10 */
        (((pe >>  0) & 1U) << 10) |   /* data10 = PE0  */
        (((pe >> 15) & 1U) << 11) |   /* data11 = PE15 */
        (((pe >>  2) & 1U) << 12) |   /* data12 = PE2  */
        (((pe >> 12) & 1U) << 13)     /* data13 = PE12 */
    ) & PLINK_ADC_MASK;
}

/*
 * Switch all 14 data pins between output (MCU drives) and input (FPGA drives).
 * Called only for write transactions; reads leave pins as input permanently.
 */
static void set_data_dir(bool output)
{
    gpio_flags_t flags = output ? GPIO_OUTPUT_INACTIVE : GPIO_INPUT;
    for (int i = 0; i < 14; i++)
        gpio_pin_configure_dt(&s_data[i], flags);
}

static int wait_not_busy(uint8_t addr, int rw)
{
    for (uint32_t elapsed = 0; gpio_pin_get_dt(&s_busy) != 0; elapsed++) {
        if (elapsed >= BUSY_TIMEOUT_US) {
            LOG_ERR("BUSY timeout (addr=0x%X rw=%d)", addr, rw);
            return -EIO;
        }
        k_busy_wait(1);
    }
    return 0;
}

/*
 * Read transaction (no lock — caller holds mutex or single-threaded).
 * Read mux in FPGA is combinational: data valid as soon as addr/rw=0 are stable.
 * No req strobe needed; bus is sampled directly after propagation delay.
 */
static uint16_t read_nolock(uint8_t addr)
{
    write_addr_rw(addr, 0);
    k_busy_wait(1);                   /* propagation: FPGA drives bus combinationally */
    return read_data_bus();
}

/*
 * Write transaction (no lock).
 * Configures data bus as output, drives 14-bit value, strobes req so the FPGA
 * latches on the rising edge, then restores data bus to input.
 */
static int write_nolock(uint8_t addr, uint16_t value)
{
    set_data_dir(true);
    write_data_bus(value);
    write_addr_rw(addr, 1);
    k_busy_wait(1);
    gpio_pin_set_dt(&s_req, 1);
    int ret = wait_not_busy(addr, 1);
    gpio_pin_set_dt(&s_req, 0);
    set_data_dir(false);
    return ret;
}

/*
 * Pulse the increment pin for one µs: rising edge advances sample-buffer,
 * new CH1/CH2 values become available combinationally on the bus.
 */
static void inc_nolock(void)
{
    gpio_pin_set_dt(&s_inc, 1);
    k_busy_wait(1);
    gpio_pin_set_dt(&s_inc, 0);
}

static uint16_t read_sample(uint8_t addr)
{
    k_mutex_lock(&s_bus_mutex, K_FOREVER);
    uint16_t result = read_nolock(addr);
    k_mutex_unlock(&s_bus_mutex);
    return result;
}

static int write_reg(uint8_t addr, uint16_t value)
{
    k_mutex_lock(&s_bus_mutex, K_FOREVER);
    int ret = write_nolock(addr, value);
    k_mutex_unlock(&s_bus_mutex);
    return ret;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

int parallel_link_init(void)
{
    int ret;
    k_mutex_init(&s_bus_mutex);

    if (!gpio_is_ready_dt(&s_areset_n)) { LOG_ERR("ARESET_N not ready"); return -ENODEV; }
    ret = gpio_pin_configure_dt(&s_areset_n, GPIO_OUTPUT_ACTIVE);
    if (ret) return ret;
    gpio_pin_set_dt(&s_areset_n, 1);

    for (int i = 0; i < 3; i++) {
        if (!gpio_is_ready_dt(&s_ctrl[i])) { LOG_ERR("CTRL[%d] not ready", i); return -ENODEV; }
        ret = gpio_pin_configure_dt(&s_ctrl[i], GPIO_OUTPUT_INACTIVE);
        if (ret) return ret;
    }

    if (!gpio_is_ready_dt(&s_rw)) { LOG_ERR("RW not ready"); return -ENODEV; }
    ret = gpio_pin_configure_dt(&s_rw, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    if (!gpio_is_ready_dt(&s_req)) { LOG_ERR("REQ not ready"); return -ENODEV; }
    ret = gpio_pin_configure_dt(&s_req, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    if (!gpio_is_ready_dt(&s_inc)) { LOG_ERR("INC not ready"); return -ENODEV; }
    ret = gpio_pin_configure_dt(&s_inc, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    /* Data bus starts as input — FPGA drives it during reads */
    for (int i = 0; i < 14; i++) {
        if (!gpio_is_ready_dt(&s_data[i])) { LOG_ERR("DATA[%d] not ready", i); return -ENODEV; }
        ret = gpio_pin_configure_dt(&s_data[i], GPIO_INPUT);
        if (ret) return ret;
    }

    if (!gpio_is_ready_dt(&s_busy)) { LOG_ERR("BUSY not ready"); return -ENODEV; }
    ret = gpio_pin_configure_dt(&s_busy, GPIO_INPUT);
    if (ret) return ret;

#ifdef CONFIG_PLINK_DEBUG
    uint8_t build = (uint8_t)(read_nolock(PLINK_ADDR_BUILD)   & 0xFFU);
    uint8_t ver   = (uint8_t)(read_nolock(PLINK_ADDR_VERSION) & 0xFFU);
    if (build != PLINK_BUILD_EXPECTED || ver != PLINK_VERSION_EXPECTED) {
        LOG_ERR("FPGA ID mismatch: build=0x%02X ver=0x%02X", build, ver);
        return -EIO;
    }
    LOG_INF("FPGA ID OK (build=0x%02X ver=0x%02X)", build, ver);
#endif

    LOG_INF("Parallel link initialized");
    return 0;
}

int parallel_link_write(uint8_t addr, uint16_t value)
{
    LOG_DBG("WRITE addr=0x%X val=0x%04X", addr, value);
    return write_reg(addr, value);
}

uint8_t parallel_link_read_byte(uint8_t addr)
{
    return (uint8_t)(read_sample(addr) & 0xFFU);
}

uint16_t parallel_link_read_sample(uint8_t addr)
{
    return read_sample(addr);
}

uint8_t parallel_link_read_status(void)
{
    return (uint8_t)(read_sample(PLINK_ADDR_STATUS) & 0xFFU);
}

int parallel_link_reset(void)
{
    int ret = gpio_pin_set_dt(&s_areset_n, 0);
    if (ret) { LOG_ERR("Failed to assert reset: %d", ret); return ret; }
    k_busy_wait(RESET_PULSE_US);
    ret = gpio_pin_set_dt(&s_areset_n, 1);
    if (ret) { LOG_ERR("Failed to de-assert reset: %d", ret); return ret; }
    k_busy_wait(100);
    LOG_INF("FPGA reset pulsed");
    return 0;
}

int parallel_link_set_sample_size(uint16_t count)
{
    if (count == 0 || count > 8192U || (count & (count - 1U)) != 0U) {
        LOG_ERR("sample_size: %u is not a power-of-2 in [1, 8192]", count);
        return -EINVAL;
    }
    uint8_t exp = 0;
    uint16_t v  = count;
    while (v > 1U) { v >>= 1; exp++; }
    return parallel_link_write(PLINK_ADDR_SAMPLE_SIZE, exp);
}

int parallel_link_acquire(uint16_t *ch1, uint16_t *ch2, uint16_t count,
                          uint32_t timeout_ms)
{
    int64_t  deadline   = k_uptime_get() + (int64_t)timeout_ms;
    uint32_t poll_count = 0;

    while (true) {
        uint8_t status = (uint8_t)(read_nolock(PLINK_ADDR_STATUS) & 0xFFU);
        if (poll_count % 1000U == 0U)
            LOG_DBG("acquire: poll=%u STATUS=0x%02x", poll_count, status);
        poll_count++;
        if (status & PLINK_STATUS_BATCH_RDY) break;
        if (k_uptime_get() >= deadline) {
            LOG_ERR("acquire: timeout after %u ms (STATUS=0x%02x)", timeout_ms, status);
            return -ETIMEDOUT;
        }
        k_sleep(K_MSEC(1));
    }

    /*
     * Bulk read using the increment pin:
     *   1. Read CH1 and CH2 combinationally (FPGA drives bus, no req needed).
     *   2. Pulse inc → FPGA advances sample-buffer pointer, latches next pair.
     *   3. Repeat for all samples.
     * Hold mutex for the whole loop to avoid interleaved transactions.
     */
    k_mutex_lock(&s_bus_mutex, K_FOREVER);
    for (uint16_t i = 0; i < count; i++) {
        ch1[i] = read_nolock(PLINK_ADDR_CH1);
        ch2[i] = read_nolock(PLINK_ADDR_CH2);
        if (i < count - 1U)
            inc_nolock();   /* advance to next sample pair */
    }
    k_mutex_unlock(&s_bus_mutex);

    return 0;
}
