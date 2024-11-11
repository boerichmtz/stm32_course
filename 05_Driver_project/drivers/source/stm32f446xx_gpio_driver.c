/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Oct 23, 2024
 *      Author: erick
 */

#include "stm32f446xx_gpio_driver.h"

/**
 * @brief   -This function enables or disable peripheral clocks
 */
void GPIO_Peripheral_Clock_Control( GPIO_Register_Definition_Type *GPIOx, uint8_t status)
{
    if (status == ENABLE)
    {
        if(GPIOx == GPIOA)
        {
            RCC_AHB1ENR_GPIOA_SET;
        }
        else if(GPIOx == GPIOB)
        {
            RCC_AHB1ENR_GPIOB_SET;
        }
        else if(GPIOx == GPIOC)
        {
            RCC_AHB1ENR_GPIOC_SET;
        }
        else if(GPIOx == GPIOD)
        {
            RCC_AHB1ENR_GPIOD_SET;
        }
        else if(GPIOx == GPIOE)
        {
            RCC_AHB1ENR_GPIOE_SET;
        }
        else if(GPIOx == GPIOF)
        {
            RCC_AHB1ENR_GPIOF_SET;
        }
        else if(GPIOx == GPIOG)
        {
            RCC_AHB1ENR_GPIOG_SET;
        }
        else if(GPIOx == GPIOH)
        {
            RCC_AHB1ENR_GPIOH_SET;
        }
    }else
    {
        if(GPIOx == GPIOA)
        {
            RCC_AHB1ENR_GPIOA_CLR;
        }
        else if(GPIOx == GPIOB)
        {
            RCC_AHB1ENR_GPIOB_CLR;
        }
        else if(GPIOx == GPIOC)
        {
            RCC_AHB1ENR_GPIOC_CLR;
        }
        else if(GPIOx == GPIOD)
        {
            RCC_AHB1ENR_GPIOD_CLR;
        }
        else if(GPIOx == GPIOE)
        {
            RCC_AHB1ENR_GPIOE_CLR;
        }
        else if(GPIOx == GPIOF)
        {
            RCC_AHB1ENR_GPIOF_CLR;
        }
        else if(GPIOx == GPIOG)
        {
            RCC_AHB1ENR_GPIOG_CLR;
        }
        else if(GPIOx == GPIOH)
        {
            RCC_AHB1ENR_GPIOH_CLR;
        }
    }
}

void set_alternate_funtion(GPIO_Handle_t *GPIOx_Handle, uint8_t register_selector, uint8_t position_in_register)
{
    if(register_selector == 1)
    {

        //Clean the register before set
        GPIOx_Handle->GPIO_x->GPIOx_AFRH &= ~((0xF) << (4*position_in_register));
        GPIOx_Handle->GPIO_x->GPIOx_AFRH &= ~((GPIOx_Handle->GPIO_Config.GPIO_PinAFR) << (4*position_in_register));
    }
    else
    {
        //Clean the register before set
        GPIOx_Handle->GPIO_x->GPIOx_AFRL &= ~((0xF) << (4*position_in_register));
        GPIOx_Handle->GPIO_x->GPIOx_AFRL &= ~((GPIOx_Handle->GPIO_Config.GPIO_PinAFR) << (4*position_in_register));
    }
}

void GPIO_Init(GPIO_Handle_t *GPIOx_Handle)
{
    //Configure the Mode for the pin
    if(GPIOx_Handle->GPIO_Config.GPIO_PinMode <= Analog_Mode)
    {
        if (GPIOx_Handle->GPIO_Config.GPIO_PinMode == Input)
        {

        }

        uint32_t mode_config = 0;
        mode_config = (GPIOx_Handle->GPIO_Config.GPIO_PinMode << (2 * GPIOx_Handle->GPIO_Config.GPIO_PinNumber));
        //clean the register
        GPIOx_Handle->GPIO_x->GPIOx_MODER &= ~(0x3 << (2 * GPIOx_Handle->GPIO_Config.GPIO_PinNumber));
        //Assign the register to the calculated value;
        GPIOx_Handle->GPIO_x->GPIOx_MODER |= mode_config;
    }
    else
    {
        RCC_APB2ENR_SYSCFG_SET;
        uint32_t mode_config = 0;
        //Pin Mode is an input Interrupt
        if (GPIOx_Handle->GPIO_Config.GPIO_PinMode == Input_Interrupt_Falling)
        {
            mode_config = (Input << (2 * GPIOx_Handle->GPIO_Config.GPIO_PinNumber));
            GPIOx_Handle->GPIO_x->GPIOx_MODER &= ~(0x03 << (2 * GPIOx_Handle->GPIO_Config.GPIO_PinNumber));
            GPIOx_Handle->GPIO_x->GPIOx_MODER |= mode_config;
            EXTI->EXTI_FTSR |= (1 << GPIOx_Handle->GPIO_Config.GPIO_PinNumber);
            //Clear rising trigger ensuring Falling TRigger is the only set
            EXTI->EXTI_RTSR &= ~(1 << GPIOx_Handle->GPIO_Config.GPIO_PinNumber);
        }
        else if (GPIOx_Handle->GPIO_Config.GPIO_PinMode == Input_Interrupt_Rising)
        {
            mode_config = (Input << (2 * GPIOx_Handle->GPIO_Config.GPIO_PinNumber));
            GPIOx_Handle->GPIO_x->GPIOx_MODER &= ~(0x03 << (2 * GPIOx_Handle->GPIO_Config.GPIO_PinNumber));
            GPIOx_Handle->GPIO_x->GPIOx_MODER |= mode_config;
            EXTI->EXTI_RTSR |= (1 << GPIOx_Handle->GPIO_Config.GPIO_PinNumber);
            //Clear falling trigger ensuring Falling TRigger is the only set
            EXTI->EXTI_FTSR &= ~(1 << GPIOx_Handle->GPIO_Config.GPIO_PinNumber);
        }
        else if (GPIOx_Handle->GPIO_Config.GPIO_PinMode == Input_Interrupt_Rising_Falling)
        {
            mode_config = (Input << (2 * GPIOx_Handle->GPIO_Config.GPIO_PinNumber));
            GPIOx_Handle->GPIO_x->GPIOx_MODER &= ~(0x03 << (2 * GPIOx_Handle->GPIO_Config.GPIO_PinNumber));
            GPIOx_Handle->GPIO_x->GPIOx_MODER |= mode_config;
            EXTI->EXTI_RTSR |= (1 << GPIOx_Handle->GPIO_Config.GPIO_PinNumber);
            EXTI->EXTI_FTSR |= (1 << GPIOx_Handle->GPIO_Config.GPIO_PinNumber);
        }

        uint8_t register_to_write; //SYSCFG_EXTICR1,2,3,4
        uint8_t offset; // EXTI 0 - 15  position 0-4

        //Calculate the register to write
        register_to_write = GPIOx_Handle->GPIO_Config.GPIO_PinNumber/4;
        offset = GPIOx_Handle->GPIO_Config.GPIO_PinNumber % 4;


        uint8_t configuration = gpio_to_port_number(GPIOx_Handle->GPIO_x);

        SYSCFG->SYSCFG_EXTICRx[register_to_write] = ( configuration << (offset * 4) ); // 4 since it is the register selection
        EXTI->EXTI_IMR |= ( 1 << GPIOx_Handle->GPIO_Config.GPIO_PinNumber);
    }

    //Configure the speed for the pin
    if(GPIOx_Handle->GPIO_Config.GPIO_PinSpeed <= HighSpeed)
    {
        uint32_t speed_config = 0;
        speed_config = (GPIOx_Handle->GPIO_Config.GPIO_PinSpeed << (2 * GPIOx_Handle->GPIO_Config.GPIO_PinNumber));
        //clean the register
        GPIOx_Handle->GPIO_x->GPIOx_OSPEEDR &= ~(0x3 << (2 * GPIOx_Handle->GPIO_Config.GPIO_PinNumber));
        //Assign the register to the calculated value;
        GPIOx_Handle->GPIO_x->GPIOx_OSPEEDR |= speed_config;
    }
    else
    {
        //interrupt
    }
    //Configure PUPDR of GPIO pin
    if(GPIOx_Handle->GPIO_Config.GPIO_PinPUPDR < reserved_PUPDR)
    {
        uint32_t pupdr_config = 0;
        pupdr_config = (GPIOx_Handle->GPIO_Config.GPIO_PinPUPDR << (2 * GPIOx_Handle->GPIO_Config.GPIO_PinNumber));
        //clean the register
        GPIOx_Handle->GPIO_x->GPIOx_PUPDR &= ~(0x3 << (2 * GPIOx_Handle->GPIO_Config.GPIO_PinNumber));
        //Assign the register to the calculated value;
        GPIOx_Handle->GPIO_x->GPIOx_PUPDR |= pupdr_config;
    }
    else
    {
        //interrupt
    }
    //Configure Output type of GPIO pin
    if(GPIOx_Handle->GPIO_Config.GPIO_PinOutput_Type <= Open_Drain)
    {
        uint32_t output_type_config = 0;
        output_type_config = (GPIOx_Handle->GPIO_Config.GPIO_PinOutput_Type << (GPIOx_Handle->GPIO_Config.GPIO_PinNumber));
        //clean the register
        GPIOx_Handle->GPIO_x->GPIOx_OTYPER &= ~(0x1 << (GPIOx_Handle->GPIO_Config.GPIO_PinNumber));
        //Assign the register to the calculated value;
        GPIOx_Handle->GPIO_x->GPIOx_OTYPER |= output_type_config;
    }
    else
    {
        //interrupt
    }
    //Configure Alternate Function of GPIO pin
    if(GPIOx_Handle->GPIO_Config.GPIO_PinMode == Alternate_Function)
    {
        if(GPIOx_Handle->GPIO_Config.GPIO_PinAFR <= AF15)
        {
            uint8_t register_selector= 0;
            uint8_t position_in_register= 0;

           register_selector = GPIOx_Handle->GPIO_Config.GPIO_PinNumber / 8;
           position_in_register = GPIOx_Handle->GPIO_Config.GPIO_PinNumber % 8;

           set_alternate_funtion(GPIOx_Handle, register_selector, position_in_register);
        }
        else
        {
            //interrupt
        }
    }
}


void RCC_AHB1RSTR_GPIOA_RST_SET_FUNC() { RCC_AHB1RSTR_GPIOA_RST_SET; }
void RCC_AHB1RSTR_GPIOA_RST_CLR_FUNC() { RCC_AHB1RSTR_GPIOA_RST_CLR; }
void RCC_AHB1RSTR_GPIOB_RST_SET_FUNC() { RCC_AHB1RSTR_GPIOB_RST_SET; }
void RCC_AHB1RSTR_GPIOB_RST_CLR_FUNC() { RCC_AHB1RSTR_GPIOB_RST_CLR; }
void RCC_AHB1RSTR_GPIOC_RST_SET_FUNC() { RCC_AHB1RSTR_GPIOC_RST_SET; }
void RCC_AHB1RSTR_GPIOC_RST_CLR_FUNC() { RCC_AHB1RSTR_GPIOC_RST_CLR; }
void RCC_AHB1RSTR_GPIOD_RST_SET_FUNC() { RCC_AHB1RSTR_GPIOD_RST_SET; }
void RCC_AHB1RSTR_GPIOD_RST_CLR_FUNC() { RCC_AHB1RSTR_GPIOD_RST_CLR; }
void RCC_AHB1RSTR_GPIOE_RST_SET_FUNC() { RCC_AHB1RSTR_GPIOE_RST_SET; }
void RCC_AHB1RSTR_GPIOE_RST_CLR_FUNC() { RCC_AHB1RSTR_GPIOE_RST_CLR; }
void RCC_AHB1RSTR_GPIOF_RST_SET_FUNC() { RCC_AHB1RSTR_GPIOF_RST_SET; }
void RCC_AHB1RSTR_GPIOF_RST_CLR_FUNC() { RCC_AHB1RSTR_GPIOF_RST_CLR; }
void RCC_AHB1RSTR_GPIOG_RST_SET_FUNC() { RCC_AHB1RSTR_GPIOG_RST_SET; }
void RCC_AHB1RSTR_GPIOG_RST_CLR_FUNC() { RCC_AHB1RSTR_GPIOG_RST_CLR; }
void RCC_AHB1RSTR_GPIOH_RST_SET_FUNC() { RCC_AHB1RSTR_GPIOH_RST_SET; }
void RCC_AHB1RSTR_GPIOH_RST_CLR_FUNC() { RCC_AHB1RSTR_GPIOH_RST_CLR; }


// Define function pointer types for set and reset operations
typedef void (*RCC_Reset_Set_Func)(void);

// Update your Reset_GPIO function to accept these pointers
void Reset_GPIO(RCC_Reset_Set_Func setFunc, RCC_Reset_Set_Func clrFunc)
{
    setFunc();    // Call the function to set the reset
    clrFunc();    // Call the function to clear the reset
}

// Update GPIO_Deinit to pass the appropriate set and clear macros
void GPIOx_Deinit(GPIO_Register_Definition_Type *GPIOx)
{
    if(GPIOx == GPIOA)
    {
        Reset_GPIO(RCC_AHB1RSTR_GPIOA_RST_SET_FUNC, RCC_AHB1RSTR_GPIOA_RST_CLR_FUNC);
    }
    else if(GPIOx == GPIOB)
    {
        Reset_GPIO(RCC_AHB1RSTR_GPIOB_RST_SET_FUNC, RCC_AHB1RSTR_GPIOB_RST_CLR_FUNC);
    }
    else if(GPIOx == GPIOC)
    {
        Reset_GPIO(RCC_AHB1RSTR_GPIOC_RST_SET_FUNC, RCC_AHB1RSTR_GPIOC_RST_CLR_FUNC);
    }
    else if(GPIOx == GPIOD)
    {
        Reset_GPIO(RCC_AHB1RSTR_GPIOD_RST_SET_FUNC, RCC_AHB1RSTR_GPIOD_RST_CLR_FUNC);
    }
    else if(GPIOx == GPIOE)
    {
        Reset_GPIO(RCC_AHB1RSTR_GPIOE_RST_SET_FUNC, RCC_AHB1RSTR_GPIOE_RST_CLR_FUNC);
    }
    else if(GPIOx == GPIOF)
    {
        Reset_GPIO(RCC_AHB1RSTR_GPIOF_RST_SET_FUNC, RCC_AHB1RSTR_GPIOF_RST_CLR_FUNC);
    }
    else if(GPIOx == GPIOG)
    {
        Reset_GPIO(RCC_AHB1RSTR_GPIOG_RST_SET_FUNC, RCC_AHB1RSTR_GPIOG_RST_CLR_FUNC);
    }
    else if(GPIOx == GPIOH)
    {
        Reset_GPIO(RCC_AHB1RSTR_GPIOH_RST_SET_FUNC, RCC_AHB1RSTR_GPIOH_RST_CLR_FUNC);
    }
}


uint8_t GPIO_Read_From_Input_Pin(GPIO_Register_Definition_Type *GPIOx, GPIO_Pin Pin_Number)
{
    uint8_t value;

    value = (uint8_t)((GPIOx->GPIOx_IDR >> Pin_Number) & 0x01);
    return value;
}

uint16_t GPIO_Read_From_Input_Port(GPIO_Register_Definition_Type *GPIOx)
{
    uint16_t port_value;

    port_value = (uint16_t) (GPIOx->GPIOx_IDR);
    return port_value;
}

void GPIO_Write_To_Output_Pin(GPIO_Register_Definition_Type *GPIOx, GPIO_Pin Pin_Number, uint8_t data)
{

    if (data == ENABLE)
    {
        GPIOx->GPIOx_BSRR |= ( 1 << Pin_Number);
    }
    else
    {
        GPIOx->GPIOx_BSRR |= (1<< (Pin_Number + 16));
    }
}


void GPIO_Write_To_Output_Port(GPIO_Register_Definition_Type *GPIOx, uint16_t data)
{
    GPIOx->GPIOx_ODR |= data;
}

void GPIO_Toggle_Output_Pin(GPIO_Register_Definition_Type *GPIOx, GPIO_Pin Pin_Number){
    GPIOx->GPIOx_ODR ^= (1 << Pin_Number);
}


void GPIO_IRQInterruptConfig(IRQn_Type IRQ_Number, uint8_t Selection)
{
    if(Selection == ENABLE)
    {
        if (IRQ_Number <= 31)
        {
            *NVIC_ISER0 |= (1 << IRQ_Number);
        }else if (IRQ_Number > 31 && IRQ_Number <= 63)
        {
            *NVIC_ISER1 |= (1 << (IRQ_Number % 32));
        }
        else if (IRQ_Number > 63 && IRQ_Number <= 95)
        {
            *NVIC_ISER2 |= (1 << (IRQ_Number % 32));
        }
    }
    else
    {
        if (IRQ_Number <= 31)
        {
            *NVIC_ICER0 = (1 << IRQ_Number);
        }else if (IRQ_Number > 31 && IRQ_Number <= 63)
        {
            *NVIC_ICER1 = (1 << (IRQ_Number % 32));
        }
        else if (IRQ_Number > 63 && IRQ_Number <= 63)
        {
            *NVIC_ICER2 = (1 << (IRQ_Number  % 32));
        }
    }
}
void GPIO_IRQPriorityConfig(IRQn_Type IRQ_Number, NVIC_Priority IRQPriority)
{
    uint8_t iprx = IRQ_Number/4;            // 4 == number of priorities by register
    uint8_t iprx_section = IRQ_Number % 4;  //
    uint8_t shift_value = (iprx_section * 8) + 4;  //

    *(NVIC_IPR_BASE + (iprx)) |= (IRQPriority << shift_value);
}


void GPIO_IRQHandling(GPIO_Pin PinNumber)
{
    if(EXTI->EXTI_PR & (1<< PinNumber))
    {
        //clear
        EXTI->EXTI_PR = (1<<PinNumber);
    }
}
