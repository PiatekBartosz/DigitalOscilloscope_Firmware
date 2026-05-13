#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <stdint.h>

#define TCP_SERVER_PORT  8888
#define WAVEFORM_SAMPLES 8192U  /* 16 KB per channel at 2 bytes/sample */

int  tcp_server_init(void);
void tcp_server_push_sample(uint16_t ch1, uint16_t ch2);

#endif /* TCP_SERVER_H */
