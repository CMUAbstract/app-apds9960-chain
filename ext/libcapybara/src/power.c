#include <msp430.h>

#include <libmsp/gpio.h>
#include <libmsp/periph.h>

#include "power.h"

void capybara_wait_for_supply()
{
    // wait for BOOST_OK: supply voltage stablized
    GPIO(LIBCAPYBARA_PORT_VBOOST_OK, IES) &= ~BIT(LIBCAPYBARA_PIN_VBOOST_OK);
    GPIO(LIBCAPYBARA_PORT_VBOOST_OK, IFG) &= ~BIT(LIBCAPYBARA_PIN_VBOOST_OK);
    GPIO(LIBCAPYBARA_PORT_VBOOST_OK, IE) |= BIT(LIBCAPYBARA_PIN_VBOOST_OK);

    while ((GPIO(LIBCAPYBARA_PORT_VBOOST_OK, IN) & BIT(LIBCAPYBARA_PIN_VBOOST_OK)) !=
                BIT(LIBCAPYBARA_PIN_VBOOST_OK)) {
        __bis_SR_register(LPM4_bits);
    }
    GPIO(LIBCAPYBARA_PORT_VBOOST_OK, IE) &= ~BIT(LIBCAPYBARA_PIN_VBOOST_OK);
    GPIO(LIBCAPYBARA_PORT_VBOOST_OK, IFG) &= ~BIT(LIBCAPYBARA_PIN_VBOOST_OK);
}

void capybara_shutdown()
{
    // Disable booster
    GPIO(LIBCAPYBARA_PORT_BOOST_SW, OUT) |= BIT(LIBCAPYBARA_PIN_BOOST_SW);

    // Sleep, while we wait for supply voltage to drop
    __disable_interrupt();
    while (1) {
        __bis_SR_register(LPM4_bits);
    }
}
