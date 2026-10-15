################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../hardware/MT6701/mt6701.c 

C_DEPS += \
./hardware/MT6701/mt6701.d 

OBJS += \
./hardware/MT6701/mt6701.o 

DIR_OBJS += \
./hardware/MT6701/*.o \

DIR_DEPS += \
./hardware/MT6701/*.d \

DIR_EXPANDS += \
./hardware/MT6701/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
hardware/MT6701/%.o: ../hardware/MT6701/%.c
	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"d:/02Projects/mounriver-studio-projects/CH32V303FOC/Debug" -I"d:/02Projects/mounriver-studio-projects/CH32V303FOC/Core" -I"d:/02Projects/mounriver-studio-projects/CH32V303FOC/User" -I"d:/02Projects/mounriver-studio-projects/CH32V303FOC/Peripheral/inc" -I"d:/02Projects/mounriver-studio-projects/CH32V303FOC/hardware/OLED/inc" -I"d:/02Projects/mounriver-studio-projects/CH32V303FOC/hardware/menu/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

