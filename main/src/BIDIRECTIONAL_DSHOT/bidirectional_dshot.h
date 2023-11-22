/**
 * @file bidirectional_dshot.h
 * @brief DMA-enabled bidirectional dshot communication protocol manager class for multiple ESCs. Adapted from teensyshot.
 * @author Jared Boyer <jaredb@mit.edu>
 * @date 22 Jul 2021
 */

#ifndef BIDIRECTIONAL_DSHOT_H_
#define BIDIRECTIONAL_DSHOT_H_

#include <Arduino.h>
#include "DMAChannel.h"

#define DSHOT_DMA_LENGTH          18            // Number of steps of one DMA sequence (the two last values are zero)
#define DSHOT_DMA_MARGIN          2             // Number of additional bit duration to wait until checking if DMA is over
#define DSHOT_DSHOT_LENGTH        16            // Number of bits in a DSHOT sequence
#define DSHOT_BT_DURATION         3330          // Duration of 1 DSHOT600 bit in ns
#define DSHOT_LP_DURATION         2500          // Duration of a DSHOT600 long pulse in ns
#define DSHOT_SP_DURATION         1250          // Duration of a DSHOT600 short pulse in ns
#define MAX_ESC                   6             // Supports up to 6 ESCs
#define TX_WAIT                   250           // Interval in microseconds at which dshot signals are sent
#define RX_WAIT                   175           // Interval in microseconds to wait for incoming receive signal? Maybe this is an accurate discription??? I think its the time it takes for an incoming erpm signal to be completely sent.
#define RX_SIGNAL_LENGTH          21            // Number of bits in the dshot feedback received signal
#define RX_BIT_TICK_LENGTH        2176          // Received signal bit pulse length in number of ARM cycle ticks (2176 ticks = 1.33 us) when overclocked to 816mhz

class DshotManager
{
public:
	DshotManager();
	void start_tx();
	void set_throttle_esc(int i, uint16_t input);
	uint32_t average_eRPM(int i);
};

void DMA_init(int i);
void assemble_signal_esc(int i);
uint32_t decode_signal(int i);
void reset_array(int i);
void tx_ISR();
void rx_ISR();

#endif /* BIDIRECTIONAL_DSHOT_H_ */