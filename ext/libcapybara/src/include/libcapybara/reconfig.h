#ifndef LIBCAPYBARA_RECONFIG_H
#define LIBCAPYBARA_RECONFIG_H

#include <stdint.h>
#include <libmsp/mem.h> 

#ifdef LIBCAPYBARA_VARTH_ENABLED
#include <libmcppot/mcp4xxx.h>
#endif // LIBCAPYBARA_VARTH_ENABLED

// The following is the lowest-level interface into the reconfigurable power
// system. The units here are bank set and wiper setting, not voltage and
// farads, which are a higher-level concept unnecessary at runtime.

#define CAPYBARA_NUM_BANKS 4
#define CAPYBARA_MAX_THRES POT_RESOLUTION // wiper settings

// Bitmask identifying a set of capacitor banks
typedef uint16_t capybara_bankmask_t;

// Bits reporting status of a precharge operation
typedef uint8_t prechg_status_t; 

extern prechg_status_t prechg_status;  

// Bits reporting status of a burst operation 
typedef uint8_t burst_status_t; 

extern burst_status_t burst_status; 

// Tuple of params that define the pwr system configuration
typedef struct {
    capybara_bankmask_t banks;
#ifdef LIBCAPYBARA_VARTH_ENABLED
    uint16_t vth;
#endif // LIBCAPYBARA_VARTH_ENABLED
} capybara_cfg_t;

extern capybara_cfg_t base_config; 
extern capybara_cfg_t prechg_config; 

// Configure the power system runtime params
int capybara_config(capybara_cfg_t cfg);

// Enable the capacitor banks specified in the bitmask
int capybara_config_banks(capybara_bankmask_t banks);

#ifdef LIBCAPYBARA_VARTH_ENABLED
// Set threshold voltage up to which to charge capacitors (units: wiper setting)
int capybara_config_threshold(uint16_t wiper);
#endif // LIBCAPYBARA_VARTH_ENABLED

// Configure settings that store maximum energy
int capybara_config_max();

// Set methods for the base power config
int set_base_banks(capybara_bankmask_t in ); 
int set_prechg_banks(capybara_bankmask_t in ); 

// Get & set methods for the precharge status
prechg_status_t get_prechg_status(void); 
int set_prechg_status(prechg_status_t); 

// Get & set methods for the burst status
burst_status_t get_burst_status(void); 
int set_burst_status(burst_status_t); 


#endif // LIBCAPYBARA_RECONFIG_H
