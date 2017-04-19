override BOARD = capybara
#override MEASURE = MEAS_PROX
export BOARD
#export MEASURE

TOOLS = \

TOOLCHAINS = \
	gcc \
	clang \

include ext/maker/Makefile
