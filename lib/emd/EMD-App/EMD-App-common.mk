################################################################################
# Copyright (c) 2017 - InvnSense Inc
#
# Common Makefile for all EMD Apps
################################################################################

#
#Look for comment "Project Specific" when recreating makefile for another project
#
SHELL := cmd.exe
# Windows
ifeq ($(OS),Windows_NT)
	DEL = del /Q
	RMDIR = rmdir /S /Q
endif

INVN_SHELLUTILS_PATH ?= %ProgramFiles(x86)%\Atmel\Studio\7.0\shellutils
$(info INVN_SHELLUTILS_PATH is $(INVN_SHELLUTILS_PATH))

MKDIR := "$(INVN_SHELLUTILS_PATH)\mkdir"
RM := "$(INVN_SHELLUTILS_PATH)\rm"

SUBMAKE_FILE := $(subst EMD-App-,,$(OUTPUT))
$(info Submake File $(EMD)/$(SUBMAKE_FILE).mk)
-include src/ASF/common_ASF_subdir.mk
-include src/$(value EMD)/$(value SUBMAKE_FILE).mk 

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := $(addprefix src/$(EMD)/,$(C_SRCS))
ifeq ($(ALGO),yes)
C_SRCS += src/algo_eapi.c
endif
C_SRCS += $(C_SRCS_ASF)
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := $(patsubst %.c,%.o, $(C_SRCS))
OBJS_AS_ARGS := 
C_DEPS := $(patsubst %.c,%.d, $(C_SRCS))
C_DEPS_AS_ARGS := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=
IDIRS += $(IDIRS_ASF)
SUBDIRS += $(SUBDIRS_ASF)
CHAR_SZ_WRN :=

$(info Source Files: $(C_SRCS))

INVN_TOOLCHAIN_PATH ?= %ProgramFiles(x86)%\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin
$(info INVN_TOOLCHAIN_PATH is $(INVN_TOOLCHAIN_PATH))

PREFIX  ?= $(INVN_TOOLCHAIN_PATH)\arm-none-eabi-
GCC     := "$(PREFIX)gcc"
OBJCOPY := "$(PREFIX)objcopy"
SIZE    := "$(PREFIX)size"
AR      := "$(PREFIX)ar"
OBJDUMP := "$(PREFIX)objdump"

ifeq ($(DEBUG),yes)
	OUTPUT_DIR  ?= Debug
else
	OUTPUT_DIR  ?= Release
endif

EXECUTABLES := $(OUTPUT_DIR)/$(OUTPUT)
ELF := elf
BIN := bin
MAP := map
HEX := hex

# Every subdirectory with source files must be described here
SUBDIRS +=  \
src/config/ \
src/$(value EMD)

IDIRS += \
src/ \
src/config \
src/$(value EMD) \
../EMD-Core \
../EMD-Core/sources \
../EMD-Core/sources/Invn/EmbUtils \
../EMD-Core/sources/EMD

PREPROCESSING_SRCS += 


ASM_SRCS += 

LIBS += \
	libarm_cortexM4lf_math_softfp \
	libm \
	libEMD-Core-$(value EMD)
	
LIBDIR += \
	src/ASF/thirdparty/CMSIS/Lib/GCC \
	../EMD-Core/bin/linaro-cm4-nucleo \
	../EMD-Core
	
ifeq ($(ALGO),yes)
LIBS += \
	libAlgoInvn \
	libMLMath
LIBDIR += ../EMD-Core/prebuilt/lib
endif

ifeq ($(ALGO_AGM),yes)
LIBS += libInvnAlgoAGM
LIBDIR += ../EMD-Core/prebuilt/lib
endif

LIBS := $(subst lib,-l,$(LIBS))

LIBDIR := $(addprefix $(QUOTE),$(LIBDIR))
LIBDIR := $(addsuffix $(QUOTE),$(LIBDIR))
LIBDIR := $(addprefix -L,$(LIBDIR))

OUTPUT_FILE_PATH +=$(OUTPUT_DIR)/$(OUTPUT).$(ELF)

OUTPUT_FILE_PATH_AS_ARGS +=$(OUTPUT_DIR)/$(OUTPUT).elf

ADDITIONAL_DEPENDENCIES:=

#OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+=  \
src/ASF/sam/utils/linker_scripts/samg/samg55j19/gcc/flash.ld

OBJS := $(addprefix $(OUTPUT_DIR)/,$(OBJS))
OBJS_AS_ARGS := $(OBJS)

C_DEPS := $(addprefix $(OUTPUT_DIR)/,$(C_DEPS))
C_DEPS_AS_ARGS := $(C_DEPS)

OUTPUT_FILE_DEP := $(addprefix $(OUTPUT_DIR)/,$(OUTPUT_FILE_DEP))

IDIRS := $(addprefix $(QUOTE),$(IDIRS))
IDIRS := $(addsuffix $(QUOTE),$(IDIRS))
IDIRS := $(addprefix -I,$(IDIRS))

FLAGS   ?= -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
WARNINGS ?= -Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int -Wmain -Wparentheses -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused \
-Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef -Wshadow -Wbad-function-cast -Wwrite-strings -Wsign-compare -Waggregate-return -Wmissing-declarations -Wformat -Wmissing-format-attribute \
-Wno-deprecated-declarations -Wpacked -Wredundant-decls -Wnested-externs -Wlong-long -Wunreachable-code -Wcast-align
CFLAGS  ?= $(FLAGS) -fdata-sections -ffunction-sections -mlong-calls -mcpu=cortex-m4 -c -pipe -fno-strict-aliasing \
-std=gnu99 -ffunction-sections -fdata-sections $(WARNINGS) --param max-inline-insns-single=500 

ifeq ($(DEBUG),yes)
	CFLAGS += -O0 -g3
else
	CFLAGS += -O0
	WARNINGS += -Wall -Wstrict-prototypes -Wmissing-prototypes -Werror-implicit-function-declaration -Wpointer-arith
endif

DEFS    +=  \
	-D__SAMG55J19__ \
	-DNDEBUG -Dscanf=iscanf \
	-DBOARD=SAMG55_XPLAINED_PRO \
	-D__SAMG55J19__ \
	-DARM_MATH_CM4=true \
	-Dprintf=iprintf \
	-DINV_MSG_ENABLE=INV_MSG_LEVEL_INFO \
	-DASSERT

ifeq ($(ALGO),yes)
CHAR_SZ_WRN := -Wl,--no-wchar-size-warning
endif

# AVR32/GNU C Compiler
define CRule
$(2): $(1)
	@-$(MKDIR) -p $(dir $(2))
	@echo Building file: $(1)
	@echo Invoking: ARM/GNU C Compiler : 6.3.1
	$(GCC)  -x c -mthumb $(DEFS) $(IDIRS)  $(CFLAGS) -MD -MP -MF "$(2:%.o=%.d)" -MT"$(2:%.o=%.d)" -MT"$(2:%.o=%.o)" -o "$(2)" "$(1)" 
	@echo Finished building: $(1)
endef
$(foreach var,$(C_SRCS), $(eval $(call CRule,$(var),$(OUTPUT_DIR)/$(dir $(var))$(notdir $(subst .c,.o,$(var)))))) 
# The above loop is used for creating object files from their respective source files. The dependent files are also created
# The Call function in the loop passes two arguments:
# C_SRCS files and their respective object files (refer OBJS)
#

SUBDIRS_AS_ARG += $(foreach var,$(SUBDIRS),$(var))
SUBDIRS_AS_ARG := $(addprefix $(OUTPUT_DIR)/,$(SUBDIRS_AS_ARG))
#SUBDIRS_AS_ARG := $(subst /,\,$(SUBDIRS_AS_ARG))- replace / with \ if using windows mkdir or del command
#leaving the above as an example/future reference 
$(info Sub Directories: $(SUBDIRS_AS_ARG))

$(OUTPUT_DIR)/src:
		-$(MKDIR) -p $(SUBDIRS_AS_ARG)
	
# AVR32/GNU Preprocessing Assembler

# AVR32/GNU Assembler

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OUTPUT_DIR)/src $(OBJS) $(USER_OBJS) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: ARM/GNU Linker : 6.3.1
	$(GCC) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS) $(USER_OBJS) $(LIBS) -mthumb -Wl,-Map="$(OUTPUT_DIR)/$(OUTPUT).map" -Wl,--start-group $(LIBS)  -Wl,--end-group $(LIBDIR) -Wl,--gc-sections -mcpu=cortex-m4 $(CHAR_SZ_WRN) -Wl,--entry=Reset_Handler -Wl,--cref -mthumb -T$(LINKER_SCRIPT_DEP) -mfloat-abi=hard -mfpu=fpv4-sp-d16  
	@echo Finished building target: $@
	$(OBJCOPY) -O binary "$(OUTPUT_DIR)/$(OUTPUT).elf" "$(OUTPUT_DIR)/$(OUTPUT).bin"
	$(OBJCOPY) -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "$(OUTPUT_DIR)/$(OUTPUT).elf" "$(OUTPUT_DIR)/$(OUTPUT).hex"
	$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O binary "$(OUTPUT_DIR)/$(OUTPUT).elf" "$(OUTPUT_DIR)/$(OUTPUT).eep" || exit 0
	$(OBJDUMP) -h -S "$(OUTPUT_DIR)/$(OUTPUT).elf" > "$(OUTPUT_DIR)/$(OUTPUT).lss"
	$(OBJCOPY) -O srec -R .eeprom -R .fuse -R .lock -R .signature  "$(OUTPUT_DIR)/$(OUTPUT).elf" "$(OUTPUT_DIR)/$(OUTPUT).srec"
	$(SIZE) "$(OUTPUT_DIR)/$(OUTPUT).elf"
	

# Other Targets
clean:
	-$(RM) -rf $(OBJS_AS_ARGS) 
	-$(RM) -rf $(C_DEPS_AS_ARGS) 
	-$(RM) -rf "$(EXECUTABLES).elf" "$(EXECUTABLES).hex" "$(EXECUTABLES).bin" "$(EXECUTABLES).lss" "$(EXECUTABLES).eep" "$(EXECUTABLES).map" "$(EXECUTABLES).srec" 
	
rebuild: clean all
