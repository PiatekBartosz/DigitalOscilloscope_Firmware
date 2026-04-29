#ifndef PARALLEL_LINK_H
#define PARALLEL_LINK_H

#include <stdbool.h>
#include <stdint.h>
#define PLINK_OP_CH1             0x0U
#define PLINK_OP_CH2             0x1U
#define PLINK_OP_STATUS          0x2U
#define PLINK_OP_CTRL_REG        0x3U
#define PLINK_OP_SAMPLE_CNT      0x4U
#define PLINK_OP_FREQUENCY       0x5U
#define PLINK_OP_STREAM          0x6U
#define PLINK_OP_RESET           0x7U

#define PLINK_CTRL_CAPTURE_EN    (1U << 0)
#define PLINK_CTRL_MOCK_EN       (1U << 1)
#define PLINK_CTRL_RESET_FIFO    (1U << 2)

#define PLINK_STATUS_FIFO_OVF    (1U << 0)
#define PLINK_STATUS_BATCH_RDY   (1U << 1)
#define PLINK_STATUS_SDRAM_BSY   (1U << 2)

#define PLINK_ADC_MASK           0x3FFFU

#define PLINK_DEVICE_ID_EXPECTED 0x0ADCu

typedef void (*plink_irq_cb_t)(uint16_t ch1, uint16_t ch2);

int parallel_link_init(plink_irq_cb_t cb);

uint16_t parallel_link_read_ch1(void);
uint16_t parallel_link_read_ch2(void);

uint16_t parallel_link_read_status(void);

int parallel_link_write(uint8_t op, uint8_t value);

uint16_t parallel_link_read(uint8_t op);

int parallel_link_stream_enable(void);
int parallel_link_stream_disable(void);

int parallel_link_reset(void);

#endif /* PARALLEL_LINK_H */
