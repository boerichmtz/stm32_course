################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/source/stm32f446xx_gpio_driver.c 

OBJS += \
./drivers/source/stm32f446xx_gpio_driver.o 

C_DEPS += \
./drivers/source/stm32f446xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/source/%.o drivers/source/%.su drivers/source/%.cyclo: ../drivers/source/%.c drivers/source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"/home/erick/STM32CubeIDE/workspace_1.16.1/05_Driver_project/drivers/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-source

clean-drivers-2f-source:
	-$(RM) ./drivers/source/stm32f446xx_gpio_driver.cyclo ./drivers/source/stm32f446xx_gpio_driver.d ./drivers/source/stm32f446xx_gpio_driver.o ./drivers/source/stm32f446xx_gpio_driver.su

.PHONY: clean-drivers-2f-source
