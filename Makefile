
BINARY = adc
OPENCM3_DIR = libopencm3
LDSCRIPT = stm32f3-nucleof334.ld
OOCD_INTERFACE = stlink-v2-1
OOCD_BOARD = st_nucleo_f3
V=1

include  libopencm3.target.mk
include  libopencm3.rules.mk
