#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "parallel_link/parallel_link.h"

#define TCP_SERVER_PORT  8888
#define WAVEFORM_SAMPLES PLINK_SAMPLE_COUNT_MAX
#define WAVEFORM_FRAME_SYNC_0 0xADU
#define WAVEFORM_FRAME_SYNC_1 0xC1U

int tcp_server_init(void);

#endif /* TCP_SERVER_H */
