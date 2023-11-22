/**
 * @file bidirectional_dshot.cpp
 * @brief DMA-enabled bidirectional dshot communication protocol manager class for multiple ESCs. Adapted from teensyshot.
 * @author Jared Boyer <jaredb@mit.edu>
 * @date 22 Jul 2021
 */

#include "bidirectional_dshot.h"

#if defined(__IMXRT1062__) // teensy 4.0
#define F_TMR F_BUS_ACTUAL
#endif

const uint16_t DSHOT_short_pulse  = uint64_t(F_TMR) * DSHOT_SP_DURATION / 1000000000;     // DSHOT short pulse duration (nb of F_BUS periods)
const uint16_t DSHOT_long_pulse   = uint64_t(F_TMR) * DSHOT_LP_DURATION / 1000000000;     // DSHOT long pulse duration (nb of F_BUS periods)
const uint16_t DSHOT_bit_length   = uint64_t(F_TMR) * DSHOT_BT_DURATION / 1000000000;     // DSHOT bit duration (nb of F_BUS periods)

static constexpr uint16_t telemetry = 0; // No telemetry request for bidirectional dshot

IntervalTimer tx_timer;
IntervalTimer rx_timer;

// ------------------------------------------------------------ //
// eFlexPWM module & DMA interactions config
// ------------------------------------------------------------ //

// DMA eFlexPWM modules
volatile IMXRT_FLEXPWM_t*  DSHOT_mods[MAX_ESC]  = { &IMXRT_FLEXPWM2,
                                                    &IMXRT_FLEXPWM1,
                                                    &IMXRT_FLEXPWM4,
                                                    &IMXRT_FLEXPWM4,
                                                    &IMXRT_FLEXPWM4,
                                                    &IMXRT_FLEXPWM2
                                                  };

// DMA eFlexPWM submodules
volatile uint8_t          DSHOT_sm[MAX_ESC]     = { 0,
                                                    3,
                                                    2,
                                                    0,
                                                    1,
                                                    2
                                                  };

// DMA eFlexPWM submodule PWM channel selector: A=0, B=1, X=2
volatile uint8_t          DSHOT_abx[MAX_ESC]    = { 0,
                                                    0,
                                                    0,
                                                    0,
                                                    0,
                                                    1
                                                  };

// Output pins
volatile uint8_t          DSHOT_pin[MAX_ESC]    = { 4,
                                                    8,
                                                    2,
                                                    22,
                                                    23,
                                                    9
                                                  };

// Output pin ALT mux
volatile uint8_t          DSHOT_pinmux[MAX_ESC] = { 1,
                                                    6,
                                                    1,
                                                    1,
                                                    1,
                                                    2
                                                  };

// DMA source
volatile uint8_t          DSHOT_dmamux[MAX_ESC] = { DMAMUX_SOURCE_FLEXPWM2_WRITE0,
                                                    DMAMUX_SOURCE_FLEXPWM1_WRITE3,
                                                    DMAMUX_SOURCE_FLEXPWM4_WRITE2,
                                                    DMAMUX_SOURCE_FLEXPWM4_WRITE0,
                                                    DMAMUX_SOURCE_FLEXPWM4_WRITE1,
                                                    DMAMUX_SOURCE_FLEXPWM2_WRITE2
                                                  };

// ------------------------------------------------------------ //
// DMA & Data init
// ------------------------------------------------------------ //

// DMA objects
DMAChannel          dma[MAX_ESC];

// DMA data
volatile uint16_t   DSHOT_dma_data[MAX_ESC][DSHOT_DMA_LENGTH];

// DSHOT data
volatile uint16_t   DSHOT_signals[MAX_ESC]; // Unassembled throttle commands from 0-2047

// ------------------------------------------------------------ //
// Dshot typedef for Rx management
// ------------------------------------------------------------ //

typedef struct {
  struct {
    void(*ISR)(void);
    volatile int timeRecord[RX_SIGNAL_LENGTH] = {0};
    volatile unsigned long lastTime;
    volatile int ISR_counter;
    unsigned long CHECKSUM_ERR_COUNTER = 0;
    unsigned long CHECKSUM_SUCCESS_COUNTER = 0;
    volatile float eRPM_sum;
    volatile uint8_t eRPM_counter;
  } line[MAX_ESC];
  bool decode_flag = false;
} DSHOT_t;

DSHOT_t dshot_comm;

// ------------------------------------------------------------ //
// Pulse length measurement ISRs for Rx
// ------------------------------------------------------------ //

#define pulselength_ISR( DSHOT_CHANNEL ) \
void pulselength_ISR_ ## DSHOT_CHANNEL( void ) { \
  if( dshot_comm.line[DSHOT_CHANNEL].ISR_counter == 0 ){ \
    dshot_comm.line[DSHOT_CHANNEL].lastTime = ARM_DWT_CYCCNT; \
    dshot_comm.line[DSHOT_CHANNEL].ISR_counter++; \
  } else { \
    dshot_comm.line[DSHOT_CHANNEL].timeRecord[ dshot_comm.line[DSHOT_CHANNEL].ISR_counter - 1 ] = ARM_DWT_CYCCNT - dshot_comm.line[DSHOT_CHANNEL].lastTime; \
    dshot_comm.line[DSHOT_CHANNEL].lastTime = ARM_DWT_CYCCNT; \
    dshot_comm.line[DSHOT_CHANNEL].ISR_counter++; \
  } \
}

pulselength_ISR( 0 );
pulselength_ISR( 1 );
pulselength_ISR( 2 );
pulselength_ISR( 3 );
pulselength_ISR( 4 );
pulselength_ISR( 5 );

void (*DSHOT_RX_ISR[6])( void ) = { pulselength_ISR_0,
                                    pulselength_ISR_1,
                                    pulselength_ISR_2,
                                    pulselength_ISR_3,
                                    pulselength_ISR_4,
                                    pulselength_ISR_5
                                  };

// ------------------------------------------------------------ //
// DMA termination ISRs
// ------------------------------------------------------------ //

#define DSHOT_DMA_interrupt_routine( DSHOT_CHANNEL ) \
void DSHOT_DMA_interrupt_routine_ ## DSHOT_CHANNEL( void ) { \
  dma[DSHOT_CHANNEL].clearInterrupt( ); \
  (*DSHOT_mods[DSHOT_CHANNEL]).MCTRL &= ~( FLEXPWM_MCTRL_RUN( 1 << DSHOT_sm[DSHOT_CHANNEL] ) );  \
  pinMode(DSHOT_pin[DSHOT_CHANNEL], INPUT); \
  attachInterrupt(DSHOT_pin[DSHOT_CHANNEL], DSHOT_RX_ISR[DSHOT_CHANNEL], CHANGE); \
}

DSHOT_DMA_interrupt_routine( 1 );
DSHOT_DMA_interrupt_routine( 2 );
DSHOT_DMA_interrupt_routine( 3 );
DSHOT_DMA_interrupt_routine( 4 );
DSHOT_DMA_interrupt_routine( 5 );

// The first ISR has to contain additional timer setup to attach an interrupt RX_WAIT time from now
void DSHOT_DMA_interrupt_routine_0( void ) {
  dma[0].clearInterrupt( );
  (*DSHOT_mods[0]).MCTRL &= ~( FLEXPWM_MCTRL_RUN( 1 << DSHOT_sm[0] ) );
  pinMode(DSHOT_pin[0], INPUT);
  attachInterrupt(DSHOT_pin[0], DSHOT_RX_ISR[0], CHANGE);
  rx_timer.begin(rx_ISR, RX_WAIT);
}

void (*DSHOT_DMA_ISR[6])( void )  = { DSHOT_DMA_interrupt_routine_0,
                                      DSHOT_DMA_interrupt_routine_1,
                                      DSHOT_DMA_interrupt_routine_2,
                                      DSHOT_DMA_interrupt_routine_3,
                                      DSHOT_DMA_interrupt_routine_4,
                                      DSHOT_DMA_interrupt_routine_5
                                    };

// ------------------------------------------------------------ //
// DshotManager Class definitions
// ------------------------------------------------------------ //

DshotManager::DshotManager()
{
  // Start ARM cycle counter
  ARM_DEMCR |= ARM_DEMCR_TRCENA; // debug exception monitor control register; enables trace and debug
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}

void DshotManager::set_throttle_esc(int i, uint16_t input)
{
  DSHOT_signals[i] = input;
}

void DshotManager::start_tx()
{
  tx_timer.begin(tx_ISR, TX_WAIT);
}

uint32_t DshotManager::average_eRPM(int i)
{
  uint32_t eRPM_average;
  if (dshot_comm.line[i].eRPM_counter != 0) { // No zero division; in case we haven't received a legitimate packet
    eRPM_average = rint( dshot_comm.line[i].eRPM_sum / dshot_comm.line[i].eRPM_counter );
  } else {
    eRPM_average = 0xffffffff;
  }
  // Reset (shared data -- no interrupts)
  noInterrupts();
  dshot_comm.line[i].eRPM_sum = 0;
  dshot_comm.line[i].eRPM_counter = 0;
  interrupts();

  return eRPM_average;
}

// ------------------------------------------------------------ //
// Non-class methods
// ------------------------------------------------------------ //

void DMA_init(int i) {

  // Configure pins on the board as DSHOT outputs
  // These pins are configured as eFlexPWM (FLEXPWMn) PWM outputs
  *(portConfigRegister( DSHOT_pin[i] )) = DSHOT_pinmux[i];

  // Configure eFlexPWM modules and submodules for PWM generation
  // --- submodule specific registers ---
  // INIT: initial counter value
  // VAL0: PWM_X compare value
  // VAL1: counter max value
  // VAL2: must be 0 for edge-aligned PWM
  // VAL3: PWM_A compare value
  // VAL4: must be 0 for edge-aligned PWM
  // VAL5: PWM_B compare value
  // OCTRL: invert polarity of PWMq FLEXPWM_SMOCTRL_POLq
  // DMAEN: FLEXPWM_SMDMAEN_VALDE to enable DMA
  // --- module specific registers ---
  // OUTEN: output enable for submodule n and PWM q FLEXPWM_OUTEN_PWMq_EN( 1 << n )
  (*DSHOT_mods[i]).SM[DSHOT_sm[i]].INIT = 0;
  (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL0 = 0;
  (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL1 = DSHOT_bit_length;
  (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL2 = 0;
  (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL3 = 0;
  (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL4 = 0;
  (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL5 = 0;
  // Invert the polarity of signals for inverted dshot
  if ( DSHOT_abx[i] == 0 ) {
    (*DSHOT_mods[i]).SM[DSHOT_sm[i]].OCTRL = FLEXPWM_SMOCTRL_POLA;
    (*DSHOT_mods[i]).OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << DSHOT_sm[i]);
  } else if ( DSHOT_abx[i] == 1 ) {
    (*DSHOT_mods[i]).SM[DSHOT_sm[i]].OCTRL = FLEXPWM_SMOCTRL_POLB;
    (*DSHOT_mods[i]).OUTEN |= FLEXPWM_OUTEN_PWMB_EN(1 << DSHOT_sm[i]);
  } else {
    /* Have yet to test if channel X PWM is a different polarity / behavior
    than channels A and B, so be wary of using channel X with this code */
    (*DSHOT_mods[i]).SM[DSHOT_sm[i]].OCTRL = FLEXPWM_SMOCTRL_POLX;
    (*DSHOT_mods[i]).OUTEN |= FLEXPWM_OUTEN_PWMX_EN(1 << DSHOT_sm[i]);
  }
  (*DSHOT_mods[i]).SM[DSHOT_sm[i]].DMAEN = FLEXPWM_SMDMAEN_VALDE;

  // Each DMA channel is linked to a unique eFlexPWM submodule
  // DMA channels are triggered by independent hardware events
  dma[i].sourceBuffer( DSHOT_dma_data[i], DSHOT_DMA_LENGTH * sizeof( uint16_t ));
  if ( DSHOT_abx[i] == 0 ) {
    dma[i].destination( (uint16_t&) (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL3 );
  } else if ( DSHOT_abx[i] == 1 ) {
    dma[i].destination( (uint16_t&) (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL5 );
  } else {
    dma[i].destination( (uint16_t&) (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL0 );
  }
  dma[i].triggerAtHardwareEvent( DSHOT_dmamux[i] );
  dma[i].interruptAtCompletion( );
  dma[i].attachInterrupt( DSHOT_DMA_ISR[i] );
  dma[i].enable();

}

// ISR helper function
void assemble_signal_esc(int i) {

  int j;
  uint16_t input = DSHOT_signals[i];

  input = (input << 1) + telemetry;
  uint16_t crc = ( ~( input ^ (input >> 4) ^ (input >> 8) ) ) & 0x0F;
  uint16_t final_sig = (input << 4) + crc;

  // Convert to array of pulse lengths
  for ( j = 0; j < DSHOT_DSHOT_LENGTH; j++ ) {

    if ( final_sig & ( 1 << ( DSHOT_DSHOT_LENGTH - 1 - j ) ) ) {
      DSHOT_dma_data[i][j] = DSHOT_long_pulse;
    } else {
      DSHOT_dma_data[i][j] = DSHOT_short_pulse;
    }

  }

  // Append trailing 0's of DMA buffer
  for ( j = DSHOT_DSHOT_LENGTH; j < DSHOT_DMA_LENGTH; j++ ) {
    DSHOT_dma_data[i][j] = 0;
  }

}

uint32_t decode_signal(int i)
{

  if (dshot_comm.line[i].timeRecord[0] == 0) { // Did we receive something?
    reset_array(i);
    return 0xffff;
  }

  int counter = 0;
  int num;
  uint32_t rx_sig = 0;

  for (int j = 0; j < RX_SIGNAL_LENGTH; j++) {
    num = rint( dshot_comm.line[i].timeRecord[j] / float(RX_BIT_TICK_LENGTH) );
    for (int k = 0; k < num; k++) {
      if (j % 2 == 1) { // If the array index is odd, which determines if it's a 1 or a 0
        rx_sig += (1 << (RX_SIGNAL_LENGTH - 1 - (counter + k)));
      }
    }
    counter += num;
  }

  // Pad 1's at the end
  for (int j = counter; j < RX_SIGNAL_LENGTH; j++) {
    rx_sig += (1 << (RX_SIGNAL_LENGTH - 1 - j));
  }

  uint32_t gcr = (rx_sig ^ (rx_sig >> 1));

#define iv 0xffffffff
  gcr &= 0xfffff;

  static const uint32_t gcr_map[32] = {
    iv, iv, iv, iv, iv, iv, iv, iv, iv, 9, 10, 11, iv, 13, 14, 15,
    iv, iv, 2, 3, iv, 5, 6, 7, iv, 0, 8, 1, iv, 4, 12, iv
  };

  uint32_t decodedValue = gcr_map[gcr & 0x1f];
  decodedValue |= gcr_map[(gcr >> 5) & 0x1f] << 4;
  decodedValue |= gcr_map[(gcr >> 10) & 0x1f] << 8;
  decodedValue |= gcr_map[(gcr >> 15) & 0x1f] << 12;

  uint32_t csum = decodedValue;
  csum = csum ^ (csum >> 8);
  csum = csum ^ (csum >> 4);

  // Checksum calculation
  if ((csum & 0xf) != 0xf) {
    // Serial.println("Checksum err"); // Enable this if you need to debug rpm information prints garbage if battery is not plugged in :(
    dshot_comm.line[i].CHECKSUM_ERR_COUNTER++;
    reset_array(i);
    return 0xffff;
  }

  dshot_comm.line[i].CHECKSUM_SUCCESS_COUNTER++;
  uint32_t value = decodedValue >> 4;
  // If we receive the max period, assume motor is not spinning
  if (value == 0x0fff) {
    reset_array(i);
    dshot_comm.line[i].eRPM_counter++;
    return 0; // idk if we should do this
  }

  value = (value & 0x01ff) << ((value & 0xfe00) >> 9);
  float eRPM = (1000000 * 60 + value / 2) / value;
  reset_array(i);

  dshot_comm.line[i].eRPM_sum += eRPM;
  dshot_comm.line[i].eRPM_counter++;
  return eRPM;

}

void reset_array(int i)
{
  for (int j = 0; j < RX_SIGNAL_LENGTH; j++) {
    dshot_comm.line[i].timeRecord[j] = 0;
  }
}

// ------------------------------------------------------------ //
// IntervalTimer ISRs
// ------------------------------------------------------------ //

// Turn on master control for each submodule being used
void tx_ISR( void )
{

  for ( int i = 0; i < MAX_ESC; i++ ) {
    DMA_init(i);
    assemble_signal_esc(i);
  }

  for ( int i = 0; i < MAX_ESC; i++ ) {
    (*DSHOT_mods[i]).MCTRL |= FLEXPWM_MCTRL_RUN( 1 << DSHOT_sm[i] );
  }

}

// Set flag to decode the signals in loop function
void rx_ISR( void )
{
  int i;
  for (i = 0; i < MAX_ESC; i++) {
    dshot_comm.line[i].ISR_counter = 0;
    detachInterrupt(DSHOT_pin[i]);
  }
  for (i = 0; i < MAX_ESC; i++) {
    decode_signal(i);
  }
  rx_timer.end(); // Wait until next tx to start rx timer again
}