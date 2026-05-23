#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stdint.h>

int app_init(void);
int app_set_mock_adc(bool enable);

/* Pulse RESET_FIFO on the FPGA: forces sample_buffer back to ST_FILLING at
 * address 0 without reading any samples.  Use before raw_write to guarantee
 * a clean capture start.  Returns 0 or negative errno. */
int app_reset_fpga_buffer(void);

/* Change the FPGA capture depth.  count must be a power of two in [1, 8192].
 * Takes effect on the next acquire.  Returns 0 or negative errno. */
int      app_set_sample_size(uint16_t count);
uint16_t app_get_sample_size(void);

/* Flush any stale buffer, wait for FPGA to fill sample pairs, then read them
 * into ch1[]/ch2[] (caller must provide buffers of at least app_get_sample_size()
 * entries).  Returns 0 on success, negative errno on failure. */
int app_acquire(uint16_t *ch1, uint16_t *ch2);

#endif /* APP_H */
