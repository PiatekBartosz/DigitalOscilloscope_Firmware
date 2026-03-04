#ifndef AFE_MANAGER_H
#define AFE_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

/* Macros */

/* Type Definitions */

/* Function Prototypes */

int afe_manager_init(void);

// TODO: do getters
int afe_manager_setGain(const uint8_t channel, const double set_percent);
int afe_manager_setOffset(const uint8_t channel, const double set_percent);

#ifdef __cplusplus
}
#endif

#endif /* AFE_MANAGER_H */
