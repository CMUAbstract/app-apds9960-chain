#ifndef LIBCAPYBARA_RECONFIG_H
#define LIBCAPYBARA_RECONFIG_H

#include <stdint.h>
#include <libmcppot/mcp4xxx.h>

// The following is the lowest-level interface into the reconfigurable power
// system. The units here are bank set and wiper setting, not voltage and
// farads, which are a higher-level concept unnecessary at runtime.

#define CAPYBARA_NUM_BANKS 4
#define CAPYBARA_MAX_THRES POT_RESOLUTION // wiper settings

// Bitmask identifying a set of capacitor banks
typedef uint16_t capybara_bankmask_t;

// Bits reporting status of a precharge operation
typedef uint8_t prechg_status_t; 

// Configure the power system runtime params
int capybara_config(capybara_bankmask_t banks, uint16_t wiper);

// Enable the capacitor banks specified in the bitmask
int capybara_config_banks(capybara_bankmask_t banks);

// Set threshold voltage up to which to charge capacitors (units: wiper setting)
int capybara_config_threshold(uint16_t wiper);

// Configure settings that store maximum energy
int capybara_config_max();

#endif // LIBCAPYBARA_RECONFIG_H
