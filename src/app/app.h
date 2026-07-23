#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stdint.h>

int app_init(void);
int app_set_mock_adc(bool enable);

int app_reset_fpga_buffer(void);

int app_set_sample_size(uint16_t count);
uint16_t app_get_sample_size(void);

int app_set_pretrigger(uint16_t count);
uint16_t app_get_pretrigger(void);

int app_set_decim_factor(uint16_t factor);
uint16_t app_get_decim_factor(void);

int app_set_trigger_mode(bool enable);
bool app_get_trigger_mode(void);
bool app_get_mock_adc(void);

uint8_t app_get_fpga_status(void);

int app_acquire(uint16_t *ch1, uint16_t *ch2);

#endif /* APP_H */
