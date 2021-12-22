################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/002_AccelOnly.c \
../Src/MPU6050_Driver.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/002_AccelOnly.o \
./Src/MPU6050_Driver.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/002_AccelOnly.d \
./Src/MPU6050_Driver.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/002_AccelOnly.o: ../Src/002_AccelOnly.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/LCD_Driver" -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/002_AccelOnly.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/MPU6050_Driver.o: ../Src/MPU6050_Driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/LCD_Driver" -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/MPU6050_Driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/LCD_Driver" -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/LCD_Driver" -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

