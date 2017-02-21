# this board has an L475 but it's very similar to L476 so use that instead
MCU_SERIES = l4
CMSIS_MCU = STM32L476xx
AF_FILE = boards/stm32l476_af.csv
LD_FILE = boards/stm32l476xg.ld
TEXT_ADDR = 0x08004000
OPENOCD_CONFIG = boards/openocd_stm32l4.cfg
MICROPY_PY_CC3100 = 1
