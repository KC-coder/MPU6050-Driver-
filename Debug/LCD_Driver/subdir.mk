################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LCD_Driver/lcd.c 

OBJS += \
./LCD_Driver/lcd.o 

C_DEPS += \
./LCD_Driver/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
LCD_Driver/lcd.o: ../LCD_Driver/lcd.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/LCD_Driver" -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"LCD_Driver/lcd.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

