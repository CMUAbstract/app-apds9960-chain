//usual libs
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
//Before periph to get away with gpio macro
#include "libmspware/driverlib.h"

//specific libmsp pieces
#include <libmsp/watchdog.h>
#include <libmsp/clock.h>
#include <libmsp/gpio.h>
#include <libmsp/periph.h>
#include <libmsp/sleep.h>
#include <libmsp/mem.h>
#include <libmsp/uart.h>
#include <libmspuartlink/uartlink.h>

//Other stuff
#include <libio/console.h>
#include <libchain/chain.h>
#include <libcapybara/capybara.h>
#include <libcapybara/reconfig.h>
#include <libcapybara/power.h>

#include "proximity.h"
#include "pins.h"
//Left here for now...
//#include <libwispbase/wisp-base.h>

#ifdef CONFIG_LIBEDB_PRINTF
#include <libedb/edb.h>
#endif

#ifdef CONFIG_EDB
#include <libedb/edb.h>
#else
#define WATCHPOINT(...)
#endif


#define INIT_TASK_DURATION_ITERS  400000
#define TASK_START_DURATION_ITERS 1600000
#define BLINK_DURATION_ITERS      400000
#define WAIT_TICK_DURATION_ITERS  300000
#define NUM_BLINKS_PER_TASK       1
#define WAIT_TICKS                3
#define NUM_PACKETS 20
#define MAX_BLE_ITER 10
#define SERIES_LEN 8
//#define MEAS_PROX

//#define CNTPWR
#define PRECHRG 1
#define FXDLRG 2
#define FXDRSP 3
#define RECFG 4

#define PWRCFG PRECHRG
#define LOG_PROX 1
#define DEFAULT_CFG 							0b1111
//#define PROX_ONLY 0

struct msg_num_iter{
  CHAN_FIELD(uint8_t, iter);
};

struct msg_self_num_iter{
  SELF_CHAN_FIELD(uint8_t, iter);
};
#define FIELD_INIT_msg_self_num_iter { \
	SELF_FIELD_INITIALIZER \
}

struct msg_flag_vals{
	CHAN_FIELD(uint8_t, flag);
	CHAN_FIELD(uint8_t, stale);
};

struct msg_self_stale_flag{
	SELF_CHAN_FIELD(uint8_t, stale);
};
#define FIELD_INIT_msg_self_stale_flag { \
	SELF_FIELD_INITIALIZER \
}

struct msg_gestures{
	CHAN_FIELD(gesture_data_t, gesture_data_sets);
};

struct msg_self_gest_data{
	CHAN_FIELD(uint16_t, num_gests);
	CHAN_FIELD_ARRAY(gesture_t, gestures, MAX_GESTS);
};
#define FIELD_INIT_msg_self_gest_data { \
	SELF_FIELD_INITIALIZER, \
	SELF_FIELD_ARRAY_INITIALIZER(MAX_GESTS), \
}

struct msg_gest_data{
	CHAN_FIELD_ARRAY(gesture_t, gestures, MAX_GESTS);
	CHAN_FIELD(uint16_t, num_gests);
};

#if PWRCFG == PRECHRG
TASK(1, task_init, CONFIGD, MEDLOWP)
TASK(2, task_sample, PREBURST, HIGHP,LOWP)
TASK(3, task_gestCapture, BURST)
// Really should rope this task into gestCapture...  gestCalc reports the
// gesture, and while it can be separate, there's no point in stopping the burst
// run it- i.e., if there isn't enough energy, configure up to 0x3 and spit out
// the answer, but I'd really rather it happened WITH the burst
TASK(4, task_gestCalc, CONFIGD, HIGHP)
TASK(5, task_BLE_estab, CONFIGD, MEDLOWP)
#elif PWRCFG == RECFG
TASK(1, task_init, CONFIGD, MEDLOWP)
TASK(2, task_sample, CONFIGD, LOWP)
TASK(3, task_gestCapture,CONFIGD, HIGHP)
TASK(4, task_gestCalc, CONFIGD, HIGHP)
TASK(5, task_BLE_estab, CONFIGD, MEDLOWP)
#else
TASK(1, task_init)
TASK(2, task_sample)
TASK(3, task_gestCapture)
TASK(4, task_gestCalc)
TASK(5, task_BLE_estab)
#endif

CHANNEL(task_init, task_gestCalc, msg_gest_data);

CHANNEL(task_sample, task_gestCapture, msg_flag_vals);

SELF_CHANNEL(task_gestCapture, msg_self_stale_flag);

CHANNEL(task_gestCapture, task_gestCalc, msg_gestures);

SELF_CHANNEL(task_gestCalc, msg_self_gest_data);

SELF_CHANNEL(task_BLE_estab, msg_self_num_iter);

CHANNEL(task_init, task_BLE_estab, msg_num_iter);


typedef enum __attribute__((packed)) {
    RADIO_CMD_SET_ADV_PAYLOAD = 0,
} radio_cmd_t;

typedef struct __attribute__((packed)) {
    radio_cmd_t cmd;
    uint8_t series[SERIES_LEN];
} radio_pkt_t;

static radio_pkt_t radio_pkt;

__nv unsigned proximity_events;
volatile unsigned work_x;

void initializeHardware(void);

/** @brief Handler for capybara power-on sequence
    TODO add this to libcapybara...
*/
void _capybara_handler(void) {
    msp_watchdog_disable();
    msp_gpio_unlock();
    __enable_interrupt();
#ifndef CNTPWR
    capybara_wait_for_supply();
    capybara_config_pins();
#endif
    GPIO(PORT_SENSE_SW, OUT) &= ~BIT(PIN_SENSE_SW);
    GPIO(PORT_SENSE_SW, DIR) |= BIT(PIN_SENSE_SW);

    GPIO(PORT_RADIO_SW, OUT) &= ~BIT(PIN_RADIO_SW);
    GPIO(PORT_RADIO_SW, DIR) |= BIT(PIN_RADIO_SW);

    GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG);
    GPIO(PORT_DEBUG, DIR) |= BIT(PIN_DEBUG);
#ifndef CNTPWR
    capybara_shutdown_on_deep_discharge();
    msp_watchdog_disable();
    msp_gpio_unlock();
    capybara_config_pins();
#endif
    msp_clock_setup();
    INIT_CONSOLE();
    __enable_interrupt();
  /*
   if(prechg_status){
    capybara_config_banks(prechg_config.banks);
    //capybara_wait_for_banks();
    //msp_sleep(30);
    capybara_wait_for_supply();
   }
  */
  /* if(burst_status == 2){
        prechg_status = 0;
        burst_status = 0;
    }*/
    PRINTF("Done handler\r\n");
#if PWRCFG == FXDLRG
    base_config.banks = HIGHP; 
#elif PWRCFG == FXDSML
    base_config.banks = LOWP; 
#endif

#ifndef CNTPWR
    capybara_config_banks(base_config.banks);
    capybara_wait_for_supply();
#endif
}

void capybara_transition()
{
    // need to explore exactly how we want BURST tasks to be followed --> should
    // we ever shutdown to reconfigure? Or should we always ride the burst wave
    // until we're out of energy?
#if (PWRCFG == PRECHRG) || (PWRCFG == RECFG)
    // Check previous burst state and register a finished burst
    if(burst_status){
        burst_status = 2;
    }
    task_cfg_spec_t curpwrcfg = curctx->task->spec_cfg;
    switch(curpwrcfg){
        case BURST:
            prechg_status = 0;
            capybara_config_banks(prechg_config.banks);
            burst_status = 1;
            break;

        case PREBURST:
            if(!prechg_status){
                prechg_config.banks = curctx->task->precfg->banks;
                capybara_config_banks(prechg_config.banks);
                // Mark that we finished the config_banks_command
                prechg_status = 1;
                capybara_shutdown();
                capybara_wait_for_supply();
            }
            //intentional fall through

        case CONFIGD:
            if(base_config.banks != curctx->task->opcfg->banks){
                base_config.banks = curctx->task->opcfg->banks;
                capybara_config_banks(base_config.banks);
                capybara_wait_for_supply();
            }
            //Another intentional fall through

        default:
            break;
    }
#endif
   LOG("Running task %u \r\n",curctx->task->idx);

}

void init()
{
  _capybara_handler();

}

/**
 * @brief represents the gesture captured
 * @details DIR_NONE =  111
 						DIR_LEFT =  001
						DIR_RIGHT = 010
						DIR_UP =    011
						DIR_DOWN =  100
		These values will get held for a little while, and then revert to 000.
    Note: DON'T USE THIS FUNCTION ON CAPYBARA
*/


void encode_IO(gest_dir val){
	switch(val){
		case DIR_NONE:
			GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_1);
			GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_2);
			GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_3);
			break;
		case DIR_LEFT:
			GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_1);
			GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_2);
			GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_3);
			break;
      case DIR_RIGHT:
			GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_1);
			GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_2);
			GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_3);
			break;
		case DIR_UP:
			GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_1);
			GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_2);
      GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_3);
      break;
		case DIR_DOWN:
			GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_1);
			GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_2);
			GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_3);
			break;
	}
	delay(GESTURE_HOLD_TIME);
		GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_1);
		GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_2);
		GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_3);

	return;
}

void i2c_setup(void) {
  /*
  * Select Port 1
  * Set Pin 6, 7 to input Secondary Module Function:
  *   (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL)
  */
  GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P1,
    GPIO_PIN6 + GPIO_PIN7,
    GPIO_SECONDARY_MODULE_FUNCTION
  );

  EUSCI_B_I2C_initMasterParam param = {0};
  param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
  param.i2cClk = CS_getSMCLK();
  param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS;
  param.byteCounterThreshold = 0;
  param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;

  EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);
}

void delay(uint32_t cycles)
{
    unsigned i;
    for (i = 0; i < cycles / (1U << 15); ++i)
        __delay_cycles(1U << 15);
}

void initializeHardware()
{		//LOG("Starting HW setup \r\n");

#ifdef USE_PHOTORES
    /*
        P3SEL0 |= BIT1;
        P3SEL1 |= BIT1;
        PM5CTL0 &= ~LOCKLPM5;
    */
#endif

#ifndef  USE_PHOTORES
    i2c_setup();
		//LOG("i2c setup done \r\n");
		/*Iinitialize apds*/
		proximity_init();
		/*Now enable the proximity sensor*/
    enableProximitySensor();
#ifndef PROX_ONLY
        enableGesture();
        disableGesture();
#endif
#endif
		//LOG("APDS TEST v1:  curtsk %u\r\n", curctx->task->idx);
}

__nv uint16_t base_val;

void task_init()
{   //base_config.banks = 0x1;
    capybara_transition();
    proximity_events = 0;
    burst_status = 0;
    prechg_status = 0;
    prechg_config.banks = 0x0;
    base_config.banks = 0x0;

    task_prologue();
    //LOG("init\r\n");
		//Init task_gestCalc fields
#ifdef USE_PHOTORES
    enable_photoresistor();
    uint16_t output = 0;
    for(int i = 0; i < 16; i++){
        output += read_photoresistor();
        LOG("output = %u \r\n", output);
   }
    base_val = output >> 4;
#endif
    uint8_t i;
		for(i = 0; i < MAX_GESTS; i++){
			gest_dir  dataInit = DIR_NONE;
			CHAN_OUT1(gest_dist, gestures[i], dataInit, CH(task_init, task_gestCalc));
		}
		uint16_t gestInit = 0;
		CHAN_OUT1(uint16_t, num_gests, gestInit, CH(task_init, task_gestCalc));
		uint8_t iterInit = 0;
    CHAN_OUT1(uint8_t, iter, iterInit, CH(task_init, task_BLE_estab));

    TRANSITION_TO(task_sample);
}

void task_BLE_estab()
{   capybara_transition();
    task_prologue();
    radio_pkt.cmd = RADIO_CMD_SET_ADV_PAYLOAD;


    uint8_t num_iter = *CHAN_IN2(uint8_t, iter, SELF_IN_CH(task_BLE_estab),
                                            CH(task_init, task_BLE_estab));
    num_iter++;
    CHAN_OUT1(uint8_t, iter, num_iter, SELF_OUT_CH(task_BLE_estab));

    for(int i = 0; i < 8; i++)
      radio_pkt.series[i] = 0xB;

    LOG("Establishing bluetooth connection iter %u \r\n", num_iter);

    GPIO(PORT_RADIO_SW, OUT) |= BIT(PIN_RADIO_SW);
    for(int j = 0; j < 5; j++){
        uartlink_open_tx();
        uartlink_send((uint8_t *)&radio_pkt,sizeof(radio_pkt.cmd) + 8);
        uartlink_close();
        msp_sleep(1024);
    }
    GPIO(PORT_RADIO_SW, OUT) &= ~BIT(PIN_RADIO_SW);

    LOG("Done!\r\n");

    if(num_iter > MAX_BLE_ITER)
        TRANSITION_TO(task_sample);
    else
        TRANSITION_TO(task_BLE_estab);
}

void task_sample()
{
  capybara_transition();
  task_prologue();

//TODO double check that it's not a problem to use the apds output as an int16_t...
int16_t proxVal = 0;
#ifdef USE_PHOTORES
    enable_photoresistor();
//while(1){
    proxVal = read_photoresistor();
#else
    proxVal = readProximity();
    delay(240000);
#endif

#if LOG_PROX
    PRINTF("proxVal = %i baseval = %i\r\n",proxVal, base_val);
    //PRINTF("proxVal = %i stats:%i %i\r\n",proxVal,burst_status, prechg_status);
#endif

    //Hijack the code here if we're only running proximity sensing.
#ifdef PROX_ONLY
        TRANSITION_TO(task_sample);
#endif

uint8_t flag = 0;
	if(proxVal > /*base_val + 150*/ ALERT_THRESH){
    proximity_events++;
    //GPIO(PORT_LED_1, OUT) |= BIT(PIN_LED_1);
		flag = 1;
		uint8_t stale = 0;
		CHAN_OUT1(uint8_t, flag, flag, CH(task_sample, task_gestCapture));
		CHAN_OUT1(uint8_t, stale, stale, CH(task_sample, task_gestCapture));

    P3OUT |= BIT6;

		TRANSITION_TO(task_gestCapture);
	}
//}
else{
  //P3OUT |= BIT6;
  TRANSITION_TO(task_sample);

	}

}

void task_gestCapture()
{   capybara_transition();
    task_prologue();
    uint8_t flag = *CHAN_IN1(uint8_t, flag, CH(task_sample, task_gestCapture));
		PRINTF("Running gesture %i %i\r\n", burst_status, proximity_events);
		uint8_t stale = *CHAN_IN2(uint8_t, stale, SELF_IN_CH(task_gestCapture),
															CH(task_sample, task_gestCapture));
		stale = 1;
		/*Mark that we've started a gesture*/
		CHAN_OUT1(uint8_t, stale, stale, SELF_OUT_CH(task_gestCapture));
		uint8_t num_samps = 0;
		gesture_data_t gesture_data_;
		// Grab gesture
    if(flag > 0){
      // Turn on sensor power supply
      GPIO(PORT_SENSE_SW, OUT) |= BIT(PIN_SENSE_SW);
      msp_sleep(30 /* cycles @ ACLK=VLOCLK=~10kHz ==> ~3ms */);
      // Do preliminary init stuff
      i2c_setup();
      proximity_init();
      enableGesture();
      for(int num_attempts = 0; num_attempts < 10; num_attempts++){
          reenableGesture();

          resetGestureFields(&gesture_data_);
          //PRINTF("gestloop now!\r\n");
          int8_t gestVal = getGestureLoop(&gesture_data_, &num_samps);
          PRINTF("OUT OF GESTURE LOOP, num samps = %u, min = %u \r\n",
                    num_samps, MIN_DATA_SETS);
          if(num_samps > MIN_DATA_SETS)
              break;
      }
		}

    if(num_samps > MIN_DATA_SETS){
				CHAN_OUT1(gesture_data_t, gesture_data_sets, gesture_data_,
																											CH(task_gestCapture,task_gestCalc));
			  //LOG("transitioning to final calc!\r\n");
				TRANSITION_TO(task_gestCalc);
		}
		else{
			/*Didn't capture enough data to decide*/
			TRANSITION_TO(task_sample);
		}
}

void task_gestCalc()
{   capybara_transition();
    task_prologue();
   	PRINTF("Computing gesture... \r\n");
		gesture_data_t gest_vals = *CHAN_IN1(gesture_data_t, gesture_data_sets,
																								CH(task_gestCapture, task_gestCalc));
		uint8_t i,j, len = 8, num_samps = 4;
	  uint8_t radio_packet[8];
		gest_dir output = decodeGesture();
    radio_pkt.cmd = RADIO_CMD_SET_ADV_PAYLOAD;
    for(int i = 1; i < len; i++)
      radio_pkt.series[i] = output;
    radio_pkt.series[0] = 0xAA;
    for (; j < len % 16; ++j)
        LOG("%i ", (int)radio_pkt.series[j]);
    LOG("\r\n");

    PRINTF("-----Dir = %u, prox events = %u----", output, proximity_events);

    GPIO(PORT_RADIO_SW, OUT) |= BIT(PIN_RADIO_SW);
    //Add in a slight delay here to compensate for some mysterious RC delay...
    GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG);
    msp_sleep(400);
    GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG);
    uartlink_open_tx();
    uartlink_send((uint8_t *)&radio_pkt.cmd, sizeof(radio_pkt.cmd) + len);
    uartlink_close();
    // TODO: wait until radio is finished; for now, wait for 0.25sec
    GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG);
    msp_sleep(1024);
    GPIO(PORT_RADIO_SW, OUT) &= ~BIT(PIN_RADIO_SW);
    GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG);
		TRANSITION_TO(task_sample);

}

#define _THIS_PORT 2
__attribute__ ((interrupt(GPIO_VECTOR(_THIS_PORT))))
void  GPIO_ISR(_THIS_PORT) (void)
{
    switch (__even_in_range(INTVEC(_THIS_PORT), INTVEC_RANGE(_THIS_PORT))) {
#if LIBCAPYBARA_PORT_VBOOST_OK == _THIS_PORT
        case INTFLAG(LIBCAPYBARA_PORT_VBOOST_OK, LIBCAPYBARA_PIN_VBOOST_OK):
            capybara_vboost_ok_isr();
            break;
#else
#error Handler in wrong ISR: capybara_vboost_ok_isr
#endif // LIBCAPYBARA_PORT_VBOOST_OK

#if LIBCAPYBARA_PORT_VBANK_OK == _THIS_PORT
        case INTFLAG(LIBCAPYBARA_PORT_VBANK_OK, LIBCAPYBARA_PIN_VBANK_OK):
            capybara_vbank_ok_isr();
            break;
#else
#error TODO fix capybara_vbank_ok_isr to be independent
#endif // LIBCAPYBARA_PORT_VBANK_OK
    }
}
#undef _THIS_PORT
INIT_FUNC(init)
//TRANSITION_FUNC(capybara_transition)
ENTRY_TASK(task_init)

