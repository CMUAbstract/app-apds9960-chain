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

#if BOARD_MAJOR == 1 && BOARD_MINOR == 1
#include <libfxl/fxl6408.h>
#endif// BOARD_{MAJOR,MINOR}

//Other stuff
#include <libio/console.h>
#include <libchain/chain.h>
#include <libcapybara/capybara.h>
#include <libcapybara/reconfig.h>
#include <libcapybara/power.h>
//#include <libhmc/magnetometer.h>

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

#define CNTPWR
#define PRECHRG 1
#define FXDLRG 2
#define FXDRSP 3
#define RECFG 4
#define TEST 5

#define PWRCFG CNT
#define SEND_GEST 1
#define LOG_PROX 0
#define DEFAULT_CFG 							0b111
#define PROX_ONLY 0

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

struct msg_gest_cap_data{
	CHAN_FIELD(uint16_t, boot_num);
  CHAN_FIELD(gest_dir, gest_out);
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
TASK(1, task_init, CONFIGD, LOWP)
// Use MEDP2 for SE variant and MEDHIGHP for TE variant
//TASK(2, task_sample, PREBURST, MEDHIGHP,LOWP)
TASK(2, task_sample, PREBURST, MEDP2,LOWP)
TASK(3, task_gestCapture, BURST)
// Really should rope this task into gestCapture...  gestSend reports the
// gesture, and while it can be separate, there's no point in stopping the burst
// run it- i.e., if there isn't enough energy, configure up to 0x3 and spit out
// the answer, but I'd really rather it happened WITH the burst
TASK(4, task_gestSend, CONFIGD, MEDLOWP)
#elif PWRCFG == RECFG
TASK(1, task_init, CONFIGD, LOWP)
TASK(2, task_sample, CONFIGD, LOWP)
TASK(3, task_gestCapture,CONFIGD, MEDHIGHP)
TASK(4, task_gestSend, CONFIGD, MEDLOWP)
#elif 0
TASK(1, task_init,CONFIGD,MEDHIGHP)
TASK(2, task_sample,CONFIGD,MEDHIGHP)
TASK(3, task_gestCapture,CONFIGD,MEDHIGHP)
TASK(4, task_gestSend,CONFIGD,MEDHIGHP)
#else
TASK(1, task_init)
TASK(2, task_sample)
TASK(3, task_gestCapture)
TASK(4, task_gestSend)

#endif

CHANNEL(task_init, task_gestSend, msg_gest_data);

CHANNEL(task_sample, task_gestCapture, msg_flag_vals);

SELF_CHANNEL(task_gestCapture, msg_self_stale_flag);

CHANNEL(task_gestCapture, task_gestSend, msg_gest_cap_data);

SELF_CHANNEL(task_gestSend, msg_self_gest_data);

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


void set_burn_flag()
{   burn_flag = 1;
}

void burn_to_trigger()
{   //burn_flag = 0;
    GPIO(PORT_SENSE_SW, OUT) |= BIT(PIN_SENSE_SW);
    P3OUT |= BIT0;
    PRINTF("Burn\r\n");
    while(1){}
}

void initializeHardware(void);

static inline void radio_on()
{
#if BOARD_MAJOR == 1 && BOARD_MINOR == 0

#if PORT_RADIO_SW != PORT_RADIO_RST // we assume this below
#error Unexpected pin config: RAD_SW and RAD_RST not on same port
#endif // PORT_RADIO_SW != PORT_RADIO_RST

    GPIO(PORT_RADIO_SW, OUT) |= BIT(PIN_RADIO_SW) | BIT(PIN_RADIO_RST);
    GPIO(PORT_RADIO_RST, OUT) &= ~BIT(PIN_RADIO_RST);

#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
    fxl_set(BIT_RADIO_SW | BIT_RADIO_RST);
    fxl_clear(BIT_RADIO_RST);

#else // BOARD_{MAJOR,MINOR}
#error Unsupported board: do not know how to turn off radio (see BOARD var)
#endif // BOARD_{MAJOR,MINOR}
}

static inline void radio_off()
{
#if BOARD_MAJOR == 1 && BOARD_MINOR == 0
    GPIO(PORT_RADIO_SW, OUT) &= ~BIT(PIN_RADIO_SW);
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
    fxl_clear(BIT_RADIO_SW);
#else // BOARD_{MAJOR,MINOR}
#error Unsupported board: do not know how to turn on radio (see BOARD var)
#endif // BOARD_{MAJOR,MINOR}
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


/** @brief Handler for capybara power-on sequence
    TODO add this to libcapybara...
*/
void _capybara_handler(void) {
    msp_watchdog_disable();
    msp_gpio_unlock();
    __enable_interrupt();
// Don't wait if we're on continuous power
#ifndef CNTPWR
    capybara_wait_for_supply();
#if BOARD_MAJOR == 1 && BOARD_MINOR == 1
    capybara_wait_for_vcap();
#endif // BOARD_{MAJOR,MINOR}
#endif
    capybara_config_pins();
    msp_clock_setup();
// Set up deep_discharge stop
#ifndef CNTPWR
#if BOARD_MAJOR == 1 && BOARD_MINOR == 0
    capybara_shutdown_on_deep_discharge();
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
    capybara_wait_for_supply();
    if (capybara_shutdown_on_deep_discharge() == CB_ERROR_ALREADY_DEEPLY_DISCHARGED) {
        capybara_shutdown();
    }
#endif //BOARD.{MAJOR,MINOR}
#endif //CNTPWR

#if BOARD_MAJOR == 1 && BOARD_MINOR == 0
    GPIO(PORT_SENSE_SW, OUT) &= ~BIT(PIN_SENSE_SW);
    GPIO(PORT_SENSE_SW, DIR) |= BIT(PIN_SENSE_SW);

    GPIO(PORT_RADIO_SW, OUT) &= ~BIT(PIN_RADIO_SW);
    GPIO(PORT_RADIO_SW, DIR) |= BIT(PIN_RADIO_SW);

    P3OUT &= ~BIT5;
    P3DIR |= BIT5;

    P3OUT &= ~BIT0;
    P3DIR |= BIT0;
    GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG);
    GPIO(PORT_DEBUG, DIR) |= BIT(PIN_DEBUG);
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
    INIT_CONSOLE();
    //LOG2("i2c init\r\n");
    i2c_setup();

    //LOG2("fxl init\r\n");
    fxl_init();

    //LOG2("RADIO_SW\r\n");

    fxl_out(BIT_PHOTO_SW);
    fxl_out(BIT_RADIO_SW);
    fxl_out(BIT_RADIO_RST);
    fxl_out(BIT_APDS_SW);
    fxl_pull_up(BIT_CCS_WAKE);
    P3OUT &= ~(BIT5 | BIT6 | BIT7);
    P3DIR |= (BIT5 | BIT6 | BIT7);

    // SENSE_SW is present but is not electrically correct: do not use.
#else // BOARD_{MAJOR,MINOR}
#error Unsupported board: do not know what pins to configure (see BOARD var)
#endif // BOARD_{MAJOR,MINOR}


#if PWRCFG == PRECHRG || PWRCFG == TEST
   if(prechg_status){
    capybara_config_banks(prechg_config.banks);
    //capybara_wait_for_banks();
    msp_sleep(30);
    capybara_wait_for_supply();
   }
   if(burst_status == 2){
        prechg_status = 0;
        burst_status = 0;
   }
#endif //PWRCFG

#if PWRCFG == FXDLRG
    //Use MEDP2 for SE version and MEDHIGHP for TE version
    base_config.banks = MEDP2;
#elif PWRCFG == FXDSML
    base_config.banks = LOWP;
#endif

#ifndef CNTPWR
    capybara_config_banks(base_config.banks);
    capybara_wait_for_supply();
#endif
    LOG2("Gesture Test\r\n");
}

void capybara_transition()
{
    // need to explore exactly how we want BURST tasks to be followed --> should
    // we ever shutdown to reconfigure? Or should we always ride the burst wave
    // until we're out of energy?
#if (PWRCFG == PRECHRG) || (PWRCFG == RECFG) || (PWRCFG == TEST)

    // Check previous burst state and register a finished burst
    if(burst_status){
        burst_status = 2;
        return;
    }
    task_cfg_spec_t curpwrcfg = curctx->task->spec_cfg;
    switch(curpwrcfg){
        case BURST:
            if(!prechg_status){
              PRINTF("Error! Running w/out precharge!\r\n");
            }
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
                // Temp:
                P3OUT |= BIT7;
                base_config.banks = curctx->task->opcfg->banks;
                capybara_config_banks(base_config.banks);
                // Temp:
                capybara_shutdown();
                capybara_wait_for_supply();
                P3OUT &= ~BIT7;
            }
            //Another intentional fall through

        default:
            break;
    }
#endif
   //LOG("Running task %u \r\n",curctx->task->idx);

}

void init()
{
  _capybara_handler();
  LOG2("Done handler\r\n");
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
		//Init task_gestSend fields
#ifdef USE_PHOTORES
    enable_photoresistor();
    uint16_t output = 0;
    for(int i = 0; i < 16; i++){
        output += read_photoresistor();
        //LOG("output = %u \r\n", output);
   }
    base_val = output >> 4;
#endif
    uint8_t i;
		for(i = 0; i < MAX_GESTS; i++){
			gest_dir  dataInit = DIR_NONE;
			CHAN_OUT1(gest_dist, gestures[i], dataInit, CH(task_init, task_gestSend));
		}
		uint16_t gestInit = 0;
		CHAN_OUT1(uint16_t, num_gests, gestInit, CH(task_init, task_gestSend));
		uint8_t iterInit = 0;
    CHAN_OUT1(uint8_t, iter, iterInit, CH(task_init, task_BLE_estab));
    /*
    fxl_set(BIT_PHOTO_SW);
    LOG2("Set photo switch high!\r\n");
    while(1);
    */
    TRANSITION_TO(task_sample);
}

void task_sample()
{
  capybara_transition();
  task_prologue();
  int num_samps = 0, avg = 0;
//TODO double check that it's not a problem to use the apds output as an int16_t...
int16_t proxVal = 0;
#ifdef USE_PHOTORES
    //GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG);
    P3OUT |= BIT5;
    enable_photoresistor();
    proxVal = read_photoresistor();
    P3OUT &= ~BIT5;
    //GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG);
#else
    proxVal = readProximity();
    delay(240000);
#endif

#if LOG_PROX
    //GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG);
    PRINTF("proxVal = %i \r\n",proxVal);
    //GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG);
#endif

    // Hijack the code here if we're only running proximity sensing.
#if PROX_ONLY
        TRANSITION_TO(task_sample);
#endif

uint8_t flag = 0, cond = 0;
#if BOARD_MAJOR == 1 && BOARD_MINOR == 0
cond = proxVal > ALERT_THRESH;
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
cond = (proxVal < ALERT_THRESH) && proxVal > 10;
// Error check for the fxl going wonky on us
// TODO debug this!!
/*
if(proxVal == 0){
    fxl_init();
    fxl_out(BIT_PHOTO_SW);
    fxl_out(BIT_RADIO_SW);
    fxl_out(BIT_RADIO_RST);
    fxl_out(BIT_APDS_SW);
    fxl_pull_up(BIT_CCS_WAKE);
}
*/
#endif //BOARD_{MAJOR,MINOR}

  if(cond){
    proximity_events++;
    disable_photoresistor();
		flag = 1;
    PRINTF("\r\nExceeded thresh! ProxVal = %i \r\n",proxVal);
		uint8_t stale = 0;
		CHAN_OUT1(uint8_t, flag, flag, CH(task_sample, task_gestCapture));
		CHAN_OUT1(uint8_t, stale, stale, CH(task_sample, task_gestCapture));
		TRANSITION_TO(task_gestCapture);
  }
else{
    TRANSITION_TO(task_sample);
	}

}

void task_gestCapture()
{   capybara_transition();
    task_prologue();
    uint8_t flag = *CHAN_IN1(uint8_t, flag, CH(task_sample, task_gestCapture));
		//PRINTF("Running gesture %i %i\r\n", burst_status, proximity_events);
		uint8_t stale = *CHAN_IN2(uint8_t, stale, SELF_IN_CH(task_gestCapture),
															CH(task_sample, task_gestCapture));
		stale = 1;
		/*Mark that we've started a gesture*/
		CHAN_OUT1(uint8_t, stale, stale, SELF_OUT_CH(task_gestCapture));
		uint8_t num_samps = 0;
		gesture_data_t gesture_data_;
      // Turn on sensor power supply
#if BOARD_MAJOR == 1 && BOARD_MINOR == 0
      GPIO(PORT_SENSE_SW, OUT) |= BIT(PIN_SENSE_SW);
      msp_sleep(30 /* cycles @ ACLK=VLOCLK=~10kHz ==> ~3ms */);
      i2c_setup();
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
      fxl_set(BIT_APDS_SW);
      msp_sleep(30);
#endif
      //magnetometer_init();
      proximity_init();
      //enableProximitySensor();
      //enableGesture();
      //disableGesture();
      //delay(240000);
      //while(1){
      ////magnet_t temp;
      ////magnetometer_read(&temp);
      //uint8_t test = readProximity();
      //LOG("My proxVal = %u\r\n",test);
      ////                                    ,temp.x,temp.y,temp.z);
      //delay(240000);
      //}
      //msp_sleep(5000);
      enableGesture();
      //TODO play with the max number of attempts
      for(int num_attempts = 0; num_attempts < 10; num_attempts++){
          reenableGesture();

          resetGestureFields(&gesture_data_);
          int8_t gestVal = getGestureLoop(&gesture_data_, &num_samps);
          if(num_samps > MIN_DATA_SETS){
              break;
          }
      }
    // Clean up from gesture wreaking havoc on the i2c port...
    fxl_init();
    fxl_out(BIT_PHOTO_SW);
    fxl_out(BIT_RADIO_SW);
    fxl_out(BIT_RADIO_RST);
    fxl_out(BIT_APDS_SW);
    fxl_pull_up(BIT_CCS_WAKE);


    if(num_samps > MIN_DATA_SETS){
#if BOARD_MAJOR == 1 && BOARD_MINOR == 0
        GPIO(PORT_SENSE_SW, OUT) &= ~BIT(PIN_SENSE_SW);
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
        fxl_clear(BIT_APDS_SW);
#endif
				uint16_t cur_boot_num = get_numBoots();
        CHAN_OUT1(uint16_t, boot_num ,cur_boot_num, CH(task_gestCapture,task_gestSend));
			  gest_dir gest_out = decodeGesture();
        CHAN_OUT1(gest_dir, gest_out, gest_out, CH(task_gestCapture,task_gestSend));
#if SEND_GEST
        TRANSITION_TO(task_gestSend);
#else
        TRANSITION_TO(task_sample);
        PRINTF("-----Dir = %u, prox events = %u----\r\n", gest_out, proximity_events);
#endif
		}
		else{
      /*Didn't capture enough data to decide*/
			  TRANSITION_TO(task_sample);
		}
}

void task_gestSend()
{   capybara_transition();
    task_prologue();

    uint16_t gest_age = *CHAN_IN1(uint16_t, boot_num,
    																								CH(task_gestCapture, task_gestSend));
    gest_dir output = *CHAN_IN1(gest_dir, gest_out, CH(task_gestCapture, task_gestSend));
    uint8_t i,j, len = 8, num_samps = 4;
	  uint8_t radio_packet[8];
    radio_pkt.cmd = RADIO_CMD_SET_ADV_PAYLOAD;
    for(int i = 1; i < len; i++)
      radio_pkt.series[i] = output;
    radio_pkt.series[0] = 0xAA;
    // Add an extra byte to indicate the fail before sending
		if(gest_age != get_numBoots())
      radio_pkt.series[2] = 0xEE;
    PRINTF("****Dir = %u, prox events = %u****\r\n", output, proximity_events);
#if BOARD_MAJOR == 1 && BOARD_MINOR == 0
    GPIO(PORT_RADIO_SW, OUT) |= BIT(PIN_RADIO_SW);
    //Add in a slight delay here to compensate for some mysterious RC delay...
    msp_sleep(64);//though 400 is what was here previously
    uartlink_open_tx();
    uartlink_send((uint8_t *)&radio_pkt.cmd, sizeof(radio_pkt.cmd) + len);
    uartlink_close();
    msp_sleep(512); //<- Used for tests on 7.20
    GPIO(PORT_RADIO_SW, OUT) &= ~BIT(PIN_RADIO_SW);
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
    P3OUT |= BIT5;
    radio_on();
    //P3OUT |= BIT7;
    msp_sleep(64); // ~15ms @ ACLK/8
    //P3OUT &= ~BIT7;

    uartlink_open_tx();
    uartlink_send((uint8_t *)&radio_pkt.cmd, sizeof(radio_pkt.cmd) + len);
    uartlink_close();

    // TODO: wait until radio is finished; for now, wait for 0.25sec
    msp_sleep(1024);
    P3OUT &= ~BIT5;
    radio_off();
    //PRINTF("Sent radio packet!\r\n");
#else
#error Unsupported board: unknown radio config
#endif
   // msp_sleep(4096);
    TRANSITION_TO(task_sample);

}
#define _THIS_PORT 2
__attribute__ ((interrupt(GPIO_VECTOR(_THIS_PORT))))
void  GPIO_ISR(_THIS_PORT) (void)
{
    switch (__even_in_range(INTVEC(_THIS_PORT), INTVEC_RANGE(_THIS_PORT))) {
#if BOARD_MAJOR == 1 && BOARD_MINOR == 0
#if LIBCAPYBARA_PORT_VBOOST_OK == _THIS_PORT
        case INTFLAG(LIBCAPYBARA_PORT_VBOOST_OK, LIBCAPYBARA_PIN_VBOOST_OK):
            capybara_vboost_ok_isr();
            break;
#else
#error Handler in wrong ISR: capybara_vboost_ok_isr
#endif // LIBCAPYBARA_PORT_VBOOST_OK
#endif // BOARD_{MAJOR,MINOR}
    }
}
#undef _THIS_PORT

#define _THIS_PORT 3
__attribute__ ((interrupt(GPIO_VECTOR(_THIS_PORT))))
void  GPIO_ISR(_THIS_PORT) (void)
{
    switch (__even_in_range(INTVEC(_THIS_PORT), INTVEC_RANGE(_THIS_PORT))) {
#if BOARD_MAJOR == 1 && BOARD_MINOR == 1
#if LIBCAPYBARA_PORT_VBOOST_OK == _THIS_PORT
        case INTFLAG(LIBCAPYBARA_PORT_VBOOST_OK, LIBCAPYBARA_PIN_VBOOST_OK):
            capybara_vboost_ok_isr();
            break;
#else
#error Handler in wrong ISR: capybara_vboost_ok_isr
#endif // LIBCAPYBARA_PORT_VBOOST_OK
#endif // BOARD_{MAJOR,MINOR}
    }
}
#undef _THIS_PORT


INIT_FUNC(init)
ENTRY_TASK(task_init)

