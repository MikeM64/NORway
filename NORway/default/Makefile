################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := bash
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../NORway.c \
../usb_serial.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
NORway.o \
usb_serial.o


OBJS_AS_ARGS +=  \
NORway.o \
usb_serial.o


C_DEPS +=  \
NORway.d \
usb_serial.d


C_DEPS_AS_ARGS +=  \
NORway.d \
usb_serial.d


OUTPUT_FILE_PATH +=NORway.elf

OUTPUT_FILE_PATH_AS_ARGS +=NORway.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

# AVR32/GNU C Compiler





./%.o: .././%.c
	@echo Building file: $<
	avr-gcc -funsigned-char -funsigned-bitfields -DF_CPU=8000000UL  -O3 -fpack-struct -fshort-enums -Wall -c -gdwarf-2 -std=gnu99  -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -mmcu=at90usb1286   -o"$@" "$<" 
	@echo Finished building: $<



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

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP)
	@echo Building target: $@
	avr-gcc -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="NORway.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group  -mmcu=at90usb1286  
	@echo Finished building target: $@
	avr-objcopy -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "NORway.elf" "NORway.hex"
	avr-objcopy -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "NORway.elf" "NORway.eep" || exit 0
	avr-objdump -h -S "NORway.elf" > "NORway.lss"
	avr-size "NORway.elf"
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS)$(C_DEPS_AS_ARGS) $(EXECUTABLES) 
	rm -rf "NORway.elf" "NORway.a" "NORway.hex" "NORway.lss" "NORway.eep" "NORway.map"
	