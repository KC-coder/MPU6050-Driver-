################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32F407xx_ADC_driver.c \
../drivers/Src/stm32F407xx_I2C_driver.c \
../drivers/Src/stm32F407xx_RCC_driver.c \
../drivers/Src/stm32F407xx_SPI_driver.c \
../drivers/Src/stm32F407xx_TIM_driver.c \
../drivers/Src/stm32F407xx__USART_driver.c \
../drivers/Src/stm32F407xx_gpio_driver.c 

OBJS += \
./drivers/Src/stm32F407xx_ADC_driver.o \
./drivers/Src/stm32F407xx_I2C_driver.o \
./drivers/Src/stm32F407xx_RCC_driver.o \
./drivers/Src/stm32F407xx_SPI_driver.o \
./drivers/Src/stm32F407xx_TIM_driver.o \
./drivers/Src/stm32F407xx__USART_driver.o \
./drivers/Src/stm32F407xx_gpio_driver.o 

C_DEPS += \
./drivers/Src/stm32F407xx_ADC_driver.d \
./drivers/Src/stm32F407xx_I2C_driver.d \
./drivers/Src/stm32F407xx_RCC_driver.d \
./drivers/Src/stm32F407xx_SPI_driver.d \
./drivers/Src/stm32F407xx_TIM_driver.d \
./drivers/Src/stm32F407xx__USART_driver.d \
./drivers/Src/stm32F407xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/stm32F407xx_ADC_driver.o: ../drivers/Src/stm32F407xx_ADC_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/LCD_Driver" -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32F407xx_ADC_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32F407xx_I2C_driver.o: ../drivers/Src/stm32F407xx_I2C_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/LCD_Driver" -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32F407xx_I2C_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32F407xx_RCC_driver.o: ../drivers/Src/stm32F407xx_RCC_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/LCD_Driver" -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32F407xx_RCC_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32F407xx_SPI_driver.o: ../drivers/Src/stm32F407xx_SPI_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/LCD_Driver" -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32F407xx_SPI_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32F407xx_TIM_driver.o: ../drivers/Src/stm32F407xx_TIM_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/LCD_Driver" -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32F407xx_TIM_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32F407xx__USART_driver.o: ../drivers/Src/stm32F407xx__USART_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/LCD_Driver" -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32F407xx__USART_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32F407xx_gpio_driver.o: ../drivers/Src/stm32F407xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/LCD_Driver" -I"C:/Users/kamal chopra/Downloads/embedded/Project_workspce/003Accelerometer_MPU6050/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32F407xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

