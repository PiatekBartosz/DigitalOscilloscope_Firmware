#ifndef PARALLEL_LINK_H
#define PARALLEL_LINK_H

#include <stdint.h>

#define PLINK_OP_CH1          0x0U
#define PLINK_OP_CH2          0x1U
#define PLINK_OP_STATUS       0x2U
#define PLINK_OP_CTRL_REG     0x3U
#define PLINK_OP_SAMPLE_SIZE  0x4U  /* log2(count) in ctrl[6:3]; read returns actual count */
#define PLINK_OP_RESET        0x7U

#define PLINK_CTRL_CAPTURE_EN (1U << 0)
#define PLINK_CTRL_MOCK_EN    (1U << 1)
#define PLINK_CTRL_RESET_FIFO (1U << 2)

#define PLINK_STATUS_FIFO_OVF (1U << 0)
#define PLINK_STATUS_BATCH_RDY (1U << 1)
#define PLINK_STATUS_SDRAM_BSY (1U << 2)

#define PLINK_ADC_MASK           0x3FFFU
#define PLINK_DEVICE_ID_EXPECTED 0x0ADCu

int      parallel_link_init(void);
int      parallel_link_write(uint8_t op, uint8_t value);
uint16_t parallel_link_read(uint8_t op);
uint16_t parallel_link_read_status(void);
int      parallel_link_reset(void);

/* Write OP_SAMPLE_SIZE.  count must be a power of two in [1, 8192].
 * Returns 0 on success, -EINVAL if count is out of range or not a power of 2,
 * -EIO on bus timeout. */
int parallel_link_set_sample_size(uint16_t count);

/* Poll FPGA until batch_ready, then bulk-read count CH1/CH2 pairs.
 * Returns 0 on success, -ETIMEDOUT if the buffer doesn't fill within timeout_ms. */
int parallel_link_acquire(uint16_t *ch1, uint16_t *ch2, uint16_t count,
                          uint32_t timeout_ms);

#endif /* PARALLEL_LINK_H */
