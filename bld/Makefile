EXEC = gesture

OBJECTS = \
	main.o\
	proximity.o\

DEPS += \
	libchain\
	libmsp \
	libwispbase \
	libio \
	libcapybara\
	libmspware\
	libmcppot\

export LIBMSP_CLOCK_SOURCE = DCO
#export LIBMSP_DCO_FREQ = 16000000ull
export LIBMSP_DCO_FREQ = 8000000ull
export LIBMSP_SLEEP_TIMER = B.0.0

export LIBMSPSOFTUART_PORT_TX = 3.6
export LIBMSPSOFTUART_BAUDRATE = 9600
export LIBMSPSOFTUART_CLOCK_FREQ = $(LIBMSP_DCO_FREQ)
export LIBMSPSOFTUART_TIMER = B.0.5

export LIBCAPYBARA_NUM_BANKS = 4
export LIBCAPYBARA_SWITCH_DESIGN = NO
export LIBCAPYBARA_SWITCH_CONTROL = ONE_PIN

export LIBCAPYBARA_BANK_PORT_0 = J.0
export LIBCAPYBARA_BANK_PORT_1 = J.2
export LIBCAPYBARA_BANK_PORT_2 = 4.0
export LIBCAPYBARA_BANK_PORT_3 = J.4

export LIBCAPYBARA_PIN_VBOOST_OK = 2.3
export LIBCAPYBARA_PIN_BOOST_SW = 2.7

override VOLTAGE = 2400 # programming voltage in mV
export VOLTAGE

CONFIG_EDB ?= 0
#CONFIG_PRINTF_LIB ?= libedb
CONFIG_PRINTF_LIB ?= libmspconsole
#CONFIG_LIBEDB_PRINTF ?= eif
include $(MAKER_ROOT)/Makefile.console


export LIBMSP_DCO_FREQ = 8000000
