

SUBDIRS_ASF +=  \
	common/ \
	common/boards/ \
	common/services/ \
	common/services/clock/ \
	common/services/clock/samg/ \
	common/services/delay/ \
	common/services/delay/sam/ \
	common/services/ioport/ \
	common/services/ioport/sam/ \
	common/services/sleepmgr \
	common/services/serial/ \
	common/services/serial/sam_uart/ \
	common/services/sleepmgr/sam \
	common/utils/ \
	common/utils/interrupt/ \
	common/utils/stdio/ \
	common/utils/stdio/stdio_serial/ \
	sam/ \
	sam/boards/ \
	sam/boards/samg55_xplained_pro/ \
	sam/drivers/ \
	sam/drivers/efc/ \
	sam/drivers/flexcom/ \
	sam/drivers/tc \
	sam/drivers/twi \
	sam/drivers/pio \
	sam/drivers/pdc/ \
	sam/drivers/pdc/pdc_uart_example/ \
	sam/drivers/pmc/ \
	sam/drivers/rtc/ \
	sam/drivers/spi/ \
	sam/drivers/supc/ \
	sam/drivers/usart/ \
	sam/utils/ \
	sam/utils/cmsis/ \
	sam/utils/cmsis/samg/ \
	sam/utils/cmsis/samg/samg55/ \
	sam/utils/cmsis/samg/samg55/include/ \
	sam/utils/cmsis/samg/samg55/include/component/ \
	sam/utils/cmsis/samg/samg55/include/instance/ \
	sam/utils/cmsis/samg/samg55/include/pio/ \
	sam/utils/cmsis/samg/samg55/source/ \
	sam/utils/cmsis/samg/samg55/source/templates/ \
	sam/utils/cmsis/samg/samg55/source/templates/gcc/ \
	sam/utils/fpu/ \
	sam/utils/header_files/ \
	sam/utils/linker_scripts/ \
	sam/utils/linker_scripts/samg/ \
	sam/utils/linker_scripts/samg/samg55j19/ \
	sam/utils/linker_scripts/samg/samg55j19/gcc/ \
	sam/utils/make/ \
	sam/utils/preprocessor/ \
	sam/utils/syscalls/ \
	sam/utils/syscalls/gcc/ \
	thirdparty/ \
	thirdparty/CMSIS/ \
	thirdparty/CMSIS/Include/ \
	thirdparty/CMSIS/Lib/ \
	thirdparty/CMSIS/Lib/GCC

IDIRS_ASF += \
	common/boards \
	sam/utils \
	sam/utils/header_files \
	sam/utils/preprocessor \
	thirdparty/CMSIS/Include \
	thirdparty/CMSIS/Lib/GCC \
	sam/utils/fpu \
	common/utils \
	sam/utils/cmsis/samg/samg55/include \
	common/services/ioport \
	common/services/clock \
	sam/drivers/pmc \
	sam/drivers/efc \
	sam/drivers/supc \
	sam/boards/samg55_xplained_pro \
	sam/boards \
	sam/drivers/flexcom \
	common/services/delay \
	common/services/serial/sam_uart \
	common/services/serial \
	common/services/sleepmgr \
	sam/drivers/usart \
	sam/drivers/pdc \
	sam/drivers/pdc/pdc_uart_example \
	sam/drivers/spi \
	sam/drivers/rtc \
	common/utils/stdio/stdio_serial

C_SRCS_ASF +=  \
	common/services/sleepmgr/sam/sleepmgr.c \
	common/utils/stdio/read.c \
	sam/drivers/pio/pio.c \
	sam/drivers/pio/pio_handler.c \
	sam/drivers/tc/tc.c \
	sam/drivers/twi/twi.c \
	common/utils/stdio/write.c \
	sam/drivers/rtc/rtc.c \
	sam/drivers/pdc/pdc.c \
	sam/drivers/spi/spi.c \
	sam/drivers/usart/usart.c \
	common/services/serial/usart_serial.c \
	common/services/delay/sam/cycle_counter.c \
	sam/drivers/flexcom/flexcom.c \
	common/services/clock/samg/sysclk.c \
	common/utils/interrupt/interrupt_sam_nvic.c \
	sam/boards/samg55_xplained_pro/board_init.c \
	sam/drivers/efc/efc.c \
	sam/drivers/pmc/pmc.c \
	sam/drivers/pmc/sleep.c \
	sam/drivers/supc/supc.c \
	sam/utils/cmsis/samg/samg55/source/templates/gcc/startup_samg55.c \
	sam/utils/cmsis/samg/samg55/source/templates/system_samg55.c \
	sam/utils/syscalls/gcc/syscalls.c


C_SRCS_ASF := $(addprefix src/ASF/,$(C_SRCS_ASF))
IDIRS_ASF := $(addprefix src/ASF/,$(IDIRS_ASF))
SUBDIRS_ASF := $(addprefix src/ASF/,$(SUBDIRS_ASF))
