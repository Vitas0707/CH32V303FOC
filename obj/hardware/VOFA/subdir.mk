################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../hardware/VOFA/vofa.c 

C_DEPS += \
./hardware/VOFA/vofa.d 

OBJS += \
./hardware/VOFA/vofa.o 

DIR_OBJS += \
./hardware/VOFA/*.o \

DIR_DEPS += \
./hardware/VOFA/*.d \

DIR_EXPANDS += \
./hardware/VOFA/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
hardware/VOFA/%.o: ../hardware/VOFA/%.c
	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"d:/02Projects/mounriver-studio-projects/CH32V303FOC/Debug" -I"d:/02Projects/mounriver-studio-projects/CH32V303FOC/Core" -I"d:/02Projects/mounriver-studio-projects/CH32V303FOC/User" -I"d:/02Projects/mounriver-studio-projects/CH32V303FOC/Peripheral/inc" -I"d:/02Projects/mounriver-studio-projects/CH32V303FOC/hardware/OLED/inc" -I"d:/02Projects/mounriver-studio-projects/CH32V303FOC/hardware/menu/inc" -I"d:/02Projects/mounriver-studio-projects/CH32V303FOC/hardware/MT6701/inc" -I"d:/02Projects/mounriver-studio-projects/CH32V303FOC/hardware/FOC/inc" -I"d:/02Projects/mounriver-studio-projects/CH32V303FOC/hardware/VOFA" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

