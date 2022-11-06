################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/arm4_nvic.c \
../drivers/Src/stm32f407_exti.c \
../drivers/Src/stm32f407_gpio.c \
../drivers/Src/stm32f407_rcc.c \
../drivers/Src/stm32f407_rcc_periph.c \
../drivers/Src/stm32f407_sysconf.c 

OBJS += \
./drivers/Src/arm4_nvic.o \
./drivers/Src/stm32f407_exti.o \
./drivers/Src/stm32f407_gpio.o \
./drivers/Src/stm32f407_rcc.o \
./drivers/Src/stm32f407_rcc_periph.o \
./drivers/Src/stm32f407_sysconf.o 

C_DEPS += \
./drivers/Src/arm4_nvic.d \
./drivers/Src/stm32f407_exti.d \
./drivers/Src/stm32f407_gpio.d \
./drivers/Src/stm32f407_rcc.d \
./drivers/Src/stm32f407_rcc_periph.d \
./drivers/Src/stm32f407_sysconf.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/arm4_nvic.o: ../drivers/Src/arm4_nvic.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/Ahmed/Documents/STM32Cube/stm32f407driver/drivers/Inc" -I"C:/Users/Ahmed/Documents/STM32Cube/stm32f407driver/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/arm4_nvic.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32f407_exti.o: ../drivers/Src/stm32f407_exti.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/Ahmed/Documents/STM32Cube/stm32f407driver/drivers/Inc" -I"C:/Users/Ahmed/Documents/STM32Cube/stm32f407driver/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f407_exti.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32f407_gpio.o: ../drivers/Src/stm32f407_gpio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/Ahmed/Documents/STM32Cube/stm32f407driver/drivers/Inc" -I"C:/Users/Ahmed/Documents/STM32Cube/stm32f407driver/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f407_gpio.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32f407_rcc.o: ../drivers/Src/stm32f407_rcc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/Ahmed/Documents/STM32Cube/stm32f407driver/drivers/Inc" -I"C:/Users/Ahmed/Documents/STM32Cube/stm32f407driver/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f407_rcc.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32f407_rcc_periph.o: ../drivers/Src/stm32f407_rcc_periph.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/Ahmed/Documents/STM32Cube/stm32f407driver/drivers/Inc" -I"C:/Users/Ahmed/Documents/STM32Cube/stm32f407driver/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f407_rcc_periph.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32f407_sysconf.o: ../drivers/Src/stm32f407_sysconf.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/Ahmed/Documents/STM32Cube/stm32f407driver/drivers/Inc" -I"C:/Users/Ahmed/Documents/STM32Cube/stm32f407driver/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f407_sysconf.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

