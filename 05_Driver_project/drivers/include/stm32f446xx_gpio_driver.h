/*
 * stm32f446xx.gpio_driver.h
 *
 *  Created on: Oct 23, 2024
 *      Author: erick
 */

#ifndef INCLUDE_STM32F446XX_GPIO_DRIVER_H_
#define INCLUDE_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration structure
 */

typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinOutput_Type;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPUPDR;
    uint8_t GPIO_PinAFR;

}GPIO_PinConfig_type;

typedef struct
{
    GPIO_Register_Definition_Type *GPIO_x;
    GPIO_PinConfig_type          GPIO_Config;
}GPIO_Handle_t;

typedef enum
{
    GPIO_PIN_0  = 0,
    GPIO_PIN_1  = 1,
    GPIO_PIN_2  = 2,
    GPIO_PIN_3  = 3,
    GPIO_PIN_4  = 4,
    GPIO_PIN_5  = 5,
    GPIO_PIN_6  = 6,
    GPIO_PIN_7  = 7,
    GPIO_PIN_8  = 8,
    GPIO_PIN_9  = 9,
    GPIO_PIN_10 = 10,
    GPIO_PIN_11 = 11,
    GPIO_PIN_12 = 12,
    GPIO_PIN_13 = 13,
    GPIO_PIN_14 = 14,
    GPIO_PIN_15 = 15,
} GPIO_Pin;

typedef enum
{
    Input = 0,
    General_Purpose_Output = 1,
    Alternate_Function = 2,
    Analog_Mode = 3,
    Input_Interrupt_Rising = 4,
    Input_Interrupt_Falling = 5,
    Input_Interrupt_Rising_Falling=6
}GPIO_Mode;

typedef enum
{
    Push_Pull = 0,
    Open_Drain = 1
}GPIO_Output_Type;

typedef enum
{
    LowSpeed = 0,
    MediumSpeed = 1,
    FastSpeed = 2,
    HighSpeed = 3
}GPIO_Speed;

typedef enum
{
    No_pullup_No_pulldown = 0,
    Pullup = 1,
    Pulldown = 2,
    reserved_PUPDR = 3
}GPIO_PUPDR;

typedef enum
{
    AF0 = 0,
    AF1 = 1,
    AF2 = 2,
    AF3 = 3,
    AF4 = 4,
    AF5 = 5,
    AF6 = 6,
    AF7 = 7,
    AF8 = 8,
    AF9 = 9,
    AF10 = 10,
    AF11 = 11,
    AF12 = 12,
    AF13 = 13,
    AF14 = 14,
    AF15 = 15,
}GPIO_Alternate_Func;





/*********************************************************
 *
 * Supported API for the GPIOs
 *
 * *******************************************************/


void GPIO_Peripheral_Clock_Control( GPIO_Register_Definition_Type *GPIOx, uint8_t status);

/* Init and De-init */
void GPIO_Init(GPIO_Handle_t *GPIOx_Handle);
void GPIO_Deinit(GPIO_Register_Definition_Type *GPIOx);

/* Data Read and Write */
uint8_t GPIO_Read_From_Input_Pin(GPIO_Register_Definition_Type *GPIOx, GPIO_Pin Pin_Number);
uint16_t GPIO_Read_From_Input_Port(GPIO_Register_Definition_Type *GPIOx);
void GPIO_Write_To_Output_Pin(GPIO_Register_Definition_Type *GPIOx, GPIO_Pin Pin_Number, uint8_t data);
void GPIO_Write_To_Output_Port(GPIO_Register_Definition_Type *GPIOx, uint16_t data);
void GPIO_Toggle_Output_Pin(GPIO_Register_Definition_Type *GPIOx, GPIO_Pin Pin_Number);

/* Data Read and Write*/
void GPIO_IRQInterruptConfig(IRQn_Type IRQ_Number, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(IRQn_Type IRQ_Number, NVIC_Priority IRQPriority);
void GPIO_IRQHandling(GPIO_Pin PinNumber);






#endif /* INCLUDE_STM32F446XX_GPIO_DRIVER_H_ */
