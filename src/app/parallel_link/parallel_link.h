#ifndef PARALLEL_LINK_H
#define PARALLEL_LINK_H

#include <stdint.h>

/*
 * Register addresses (CFG_ADDR[2:0]).
 * Reads and writes share the same address space; rw pin selects direction.
 */
#define PLINK_ADDR_BUILD        0x0U  /* read: build number (0x89)          */
#define PLINK_ADDR_VERSION      0x1U  /* read: version      (0x07)          */
#define PLINK_ADDR_CH1          0x2U  /* read: CH1 sample [13:0]            */
#define PLINK_ADDR_CH2          0x3U  /* read: CH2 sample [13:0]            */
#define PLINK_ADDR_STATUS       0x4U  /* read: status flags                 */
#define PLINK_ADDR_CTRL         0x5U  /* write: capture_en/mock_en/rst_fifo */
#define PLINK_ADDR_SAMPLE_SIZE  0x6U  /* write: log2(sample count), 0..13   */
#define PLINK_ADDR_RESET        0x7U  /* write: restore all defaults        */

#define PLINK_CTRL_CAPTURE_EN   (1U << 0)
#define PLINK_CTRL_MOCK_EN      (1U << 1)
#define PLINK_CTRL_RESET_FIFO   (1U << 2)

/* Decimation factor packed in CTRL bits [13:3].  0 and 1 both mean no decimation. */
#define PLINK_CTRL_DECIM_SHIFT  3U
#define PLINK_CTRL_DECIM_MAX    2047U
#define PLINK_CTRL_DECIM(f)     ((uint16_t)((uint16_t)(f) & (uint16_t)PLINK_CTRL_DECIM_MAX) \
                                 << PLINK_CTRL_DECIM_SHIFT)

#define PLINK_STATUS_FIFO_OVF   (1U << 0)
#define PLINK_STATUS_BATCH_RDY  (1U << 1)
#define PLINK_STATUS_SDRAM_BSY  (1U << 2)

#define PLINK_ADC_MASK           0x3FFFU   /* 14-bit mask                   */
#define PLINK_BUILD_EXPECTED     0x89U
#define PLINK_VERSION_EXPECTED   0x07U

/*
 * Initialise GPIOs and verify FPGA identity.
 * Returns 0 on success, negative errno on failure.
 */
int parallel_link_init(void);

/*
 * Write a 14-bit value to a register.
 * Returns 0 on success, -EIO on BUSY timeout.
 */
int parallel_link_write(uint8_t addr, uint16_t value);

/*
 * Read an 8-bit register (lower 8 bits of the 14-bit bus).
 */
uint8_t parallel_link_read_byte(uint8_t addr);

/*
 * Read a 14-bit ADC sample register in one transaction.
 * Returns value masked to PLINK_ADC_MASK.
 */
uint16_t parallel_link_read_sample(uint8_t addr);

/*
 * Read the status register.
 */
uint8_t parallel_link_read_status(void);

/*
 * Pulse the FPGA asynchronous reset line.
 */
int parallel_link_reset(void);

/*
 * Configure the capture depth.  count must be a power of two in [1, 8192].
 */
int parallel_link_set_sample_size(uint16_t count);

/*
 * Poll until batch_ready, then bulk-read count CH1/CH2 sample pairs using
 * the increment pin for streaming.  Each increment pulse advances the
 * sample-buffer pointer; CH1 and CH2 are read combinationally after each step.
 *
 * Returns 0 on success, -ETIMEDOUT if batch_ready never fires within timeout_ms.
 */
int parallel_link_acquire(uint16_t *ch1, uint16_t *ch2, uint16_t count,
                          uint32_t timeout_ms);

#endif /* PARALLEL_LINK_H */
