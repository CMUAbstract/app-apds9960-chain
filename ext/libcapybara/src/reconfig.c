#include <msp430.h>

#ifdef LIBCAPYBARA_VARTH_ENABLED
#include <libmcppot/mcp4xxx.h>
#endif // LIBCAPYBARA_VARTH_ENABLED

#include <libmsp/periph.h>

#include "reconfig.h"

/* Working config and precharged config */ 
__nv capybara_cfg_t base_config; 
__nv capybara_cfg_t prechg_config; 

/* Precharge and Burst status globals */ 
__nv prechg_status_t prechg_status = 0;  
__nv burst_status_t burst_status = 0; 
volatile prechg_status_t v_prechg_status;
volatile burst_status_t v_burst_status; 

/* Leaving these simple for now... I can't see them ever getting too complicated, but who
 * knows? TODO turn these into macros so we don't have to pay for a function call every
 * single time they're accessed... 
 */

prechg_status_t get_prechg_status(void){
    return (prechg_status); 
}

int set_prechg_status(prechg_status_t in){
    prechg_status = in; 
    return 0; 
}

burst_status_t get_burst_status(void){
    return (burst_status); 
}

int set_burst_status(burst_status_t in){
    burst_status = in; 
    return 0; 
}

int set_base_banks(capybara_bankmask_t in){
    base_config.banks = in;
    return 0; 
}

int set_prechg_banks(capybara_bankmask_t in){
    prechg_config.banks = in;
    return 0; 
}

//  Command from userspace code to issue a precharge, takes bank config as
//  an argument (TODO: figure out if this is the best place for this function to
//  live... I suspect it doesn't belong back here given how closely its usage is
//  tied to libchain)
int issue_precharge(capybara_bankmask_t cfg){
    prechg_config.banks = cfg; 
    prechg_status = 1; 
    return 0; 
}

// Cycles for the latch cap to charge/discharge
#define SWITCH_TIME_CYCLES 0x1fff // charges to ~2.4v (almost full-scale); discharges to <100mV

#if defined(LIBCAPYBARA_SWITCH_CONTROL__ONE_PIN)

#define BANK_PORT_INNER(i) LIBCAPYBARA_BANK_PORT_ ## i ## _PORT
#define BANK_PORT(i) BANK_PORT_INNER(i)

#define BANK_PIN_INNER(i) LIBCAPYBARA_BANK_PORT_ ## i ## _PIN
#define BANK_PIN(i) BANK_PIN_INNER(i)

#define ACTUATE_SWITCH(i) \
        GPIO(BANK_PORT(i), DIR) |= BIT(BANK_PIN(i)); \
        __delay_cycles(SWITCH_TIME_CYCLES); \
        GPIO(BANK_PORT(i), DIR) &= ~BIT(BANK_PIN(i)); \

#elif defined(LIBCAPYBARA_SWITCH_CONTROL__TWO_PIN)

#define BANK_PORT_INNER(i, op) LIBCAPYBARA_BANK_PORT_ ## i ## _ ## op ## _PORT
#define BANK_PORT(i, op) BANK_PORT_INNER(i, op)

#define BANK_PIN_INNER(i, op) LIBCAPYBARA_BANK_PORT_ ## i ## _ ## op ## _PIN
#define BANK_PIN(i, op) BANK_PIN_INNER(i, op)

#define ACTUATE_SWITCH(i, op) \
        GPIO(BANK_PORT(i, op), DIR) |= BIT(BANK_PIN(i, op)); \
        __delay_cycles(SWITCH_TIME_CYCLES); \
        GPIO(BANK_PORT(i, op), DIR) &= ~BIT(BANK_PIN(i, op)); \

#endif // LIBCAPYBARA_SWITCH_CONTROL

#if defined(LIBCAPYBARA_SWITCH_DESIGN__NC)

#if defined(LIBCAPYBARA_SWITCH_CONTROL__TWO_PIN)

#define BANK_CONNECT(i) do { \
        GPIO(BANK_PORT(i, CLOSE), OUT) |= BIT(BANK_PIN(i, CLOSE)); \
        ACTUATE_SWITCH(i, CLOSE); \
    } while (0);

#define BANK_DISCONNECT(i) do { \
        GPIO(BANK_PORT(i, OPEN), OUT) &= ~BIT(BANK_PIN(i, OPEN)); \
        ACTUATE_SWITCH(i, OPEN); \
    } while (0);

#elif defined(LIBCAPYBARA_SWITCH_CONTROL__ONE_PIN)

#define BANK_CONNECT(i) do { \
        GPIO(BANK_PORT(i), OUT) &= ~BIT(BANK_PIN(i)); \
        ACTUATE_SWITCH(i); \
    } while (0);

#define BANK_DISCONNECT(i) do { \
        GPIO(BANK_PORT(i), OUT) |= BIT(BANK_PIN(i)); \
        ACTUATE_SWITCH(i); \
    } while (0);

#else // LIBCAPYBARA_SWITCH_CONTROL
#error Invalid value of config option: LIBCAPYBARA_SWITCH_CONTROL
#endif // LIBCAPYBARA_SWITCH_CONTROL

#elif defined(LIBCAPYBARA_SWITCH_DESIGN__NO)

#if defined(LIBCAPYBARA_SWITCH_CONTROL__TWO_PIN)
#error Not implemented: switch design NO, switch control TWO PIN
#elif defined(LIBCAPYBARA_SWITCH_CONTROL__ONE_PIN)

#define BANK_CONNECT(i) do { \
        GPIO(BANK_PORT(i), OUT) |= BIT(BANK_PIN(i)); \
        ACTUATE_SWITCH(i); \
    } while (0);

#define BANK_DISCONNECT(i) do { \
        GPIO(BANK_PORT(i), OUT) &= ~BIT(BANK_PIN(i)); \
        ACTUATE_SWITCH(i); \
    } while (0);

#endif // LIBCAPYBARA_SWITCH_CONTROL

#else // LIBCAPYBARA_SWITCH_DESIGN
#error Invalid value of config option: LIBCAPYBARA_SWITCH_DESIGN
#endif // LIBCAPYBARA_SWITCH_DESIGN

int capybara_config_banks(capybara_bankmask_t banks)
{
    // If the switches would be all on one port, we'd do this in one
    // assignment. Since on our board, they are hooked up to two
    // ports, we either have optimal code that does two assignments
    // but is not generic, or we have generic code that does four
    // assignments. We do the latter here.

    // NOTE: This is not a loop, because the pins and ports are
    // resolved at compile time. We don't want a runtime map.

#define CONFIG_BANK(i) \
    if (banks & (1 << i)) { BANK_CONNECT(i); } else { BANK_DISCONNECT(i); }

    CONFIG_BANK(0);
    CONFIG_BANK(1);
    //CONFIG_BANK(2);
    //CONFIG_BANK(3);

    return 0;
}

#ifdef LIBCAPYBARA_VARTH_ENABLED
int capybara_config_threshold(uint16_t wiper)
{
    uint16_t curr_wiper = pot_get_nv_wiper();
    if (curr_wiper != wiper) {
        pot_set_nv_wiper(wiper);
        pot_set_v_wiper(wiper); // not clear if redundant, so just in case
    } else {
        pot_set_v_wiper(wiper); // just in case
    }
    return 0;
}
#endif // LIBCAPYBARA_VARTH_ENABLED

int capybara_config(capybara_cfg_t cfg)
{
    int rc;

    rc = capybara_config_banks(cfg.banks);
    if (rc) return rc;

#ifdef LIBCAPYBARA_VARTH_ENABLED
    rc = capybara_config_threshold(cfg.vth);
    if (rc) return rc;
#endif // LIBCAPYBARA_VARTH_ENABLED

    return 0;
}

int capybara_config_max()
{
#ifdef LIBCAPYBARA_VARTH_ENABLED
    capybara_cfg_t cfg = { ~0, POT_RESOLUTION };
#else
    capybara_cfg_t cfg = { ~0 };
#endif // LIBCAPYBARA_VARTH_ENABLED
    return capybara_config(cfg);
}
