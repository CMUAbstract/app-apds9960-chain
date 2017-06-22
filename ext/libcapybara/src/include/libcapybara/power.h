#ifndef LIBCAPYBARA_POWER_H
#define LIBCAPYBARA_POWER_H

// Some error codes
typedef enum {
    CB_SUCCESS = 0,
    CB_ERROR_ALREADY_DEEPLY_DISCHARGED = 1,
} cb_rc_t;

// Sleep until MCU power supply stabilizes
void capybara_wait_for_supply();

// Wait for Vcap to recover after droop resulting from booster turning on
void capybara_wait_for_vcap();

// Function-macro for handling VBOOST_OK interrupt. Cannot be contained within
// the library since library cannot own the ISR which may need to also handle
// unrelated pins.  Defined as a macro because can only use __bi[cs]_SR
// intrinsics from ISR context (and to avoid function call overhead).
//
// Call this function-macro from the switch statement in the respective ISR:
//
//      #if LIBCAPYBARA_PORT_VBOOST_OK == 2 /* port number */
//         case INTFLAG(LIBCAPYBARA_PORT_VBOOST_OK, LIBCAPYBARA_PIN_VBOOST_OK):
//             capybara_vboost_ok_isr();
//             break;
//      #else
//      #error Handler in wrong ISR: capybara_vboost_ok_isr
//      #endif
//
#define capybara_vboost_ok_isr() __bic_SR_register_on_exit(LPM4_bits)

// Cut power to self by disabling the booster
void capybara_shutdown();

// Setup comparator to interrupt and call shutdown when Vbank drops
// below the deep discharge threshold (see config)
// Returns ERROR_ALREADY_DEEPLY_DISCHARGED if already below threshold when called.
cb_rc_t capybara_shutdown_on_deep_discharge();

#endif // LIBCAPYBARA_POWER_H
