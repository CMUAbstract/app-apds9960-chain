LIB = libcapybara

OBJECTS = \
	capybara.o \
	reconfig.o \
	power.o \

DEPS += \
	libmsp \
	libmcppot \

override SRC_ROOT = ../../src

override CFLAGS += \
	-I$(SRC_ROOT)/include \
	-I$(SRC_ROOT)/include/$(LIB) \

include ../Makefile.config
include ../Makefile.options
