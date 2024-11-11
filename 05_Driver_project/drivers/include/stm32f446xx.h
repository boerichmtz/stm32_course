/*
 * stm32f4xx.h
 *
 *  Created on: Oct 22, 2024
 *      Author: erick
 */

#ifndef INCLUDE_STM32F446XX_H_
#define INCLUDE_STM32F446XX_H_

#include <stdint.h>
/************************************* Generic Macro definition **************************************/
#define ENABLE 1
#define DISABLE  0


/************************************* CORTEX M4 Processor Specific**************************************/
//Interrupt Set Enable Registers
#define  NVIC_ISER0         ((volatile uint32_t*) 0xE000E100U)
#define  NVIC_ISER1         ((volatile uint32_t*) 0xE000E104U)
#define  NVIC_ISER2         ((volatile uint32_t*) 0xE000E108U)
//Interrupt Clear Enable Registers
#define  NVIC_ICER0         ((volatile uint32_t*) 0xE000E180U)
#define  NVIC_ICER1         ((volatile uint32_t*) 0xE000E184U)
#define  NVIC_ICER2         ((volatile uint32_t*) 0xE000E188U)

#define NVIC_IPR_BASE       ((volatile uint32_t*) 0xE000E400U)


/************************************* Memory Map **************************************/


#define SRAM1_BASE            0x20000000UL /*!< SRAM1(112 KB) base address in the alias region                             */
#define SRAM2_BASE            0x2001C000UL /*!< SRAM2(16 KB) base address in the alias region                              */
#define PERIPHERALS_BASE      0x40000000UL /*!< Peripheral base address in the alias region                                */

// Buses Address Definitions
#define APB1PERIPH_BASE       PERIPHERALS_BASE
#define APB2PERIPH_BASE       (PERIPHERALS_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPHERALS_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPHERALS_BASE + 0x10000000UL)
#define AHB3PERIPH_BASE       (PERIPHERALS_BASE + 0x20000000UL)

//APB1 Peripheral definitions
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800UL)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00UL)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000UL)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400UL)
#define TIM12_BASE            (APB1PERIPH_BASE + 0x1800UL)
#define TIM13_BASE            (APB1PERIPH_BASE + 0x1C00UL)
#define TIM14_BASE            (APB1PERIPH_BASE + 0x2000UL)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800UL)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00UL)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000UL)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00UL)
#define SPDIFRX_BASE          (APB1PERIPH_BASE + 0x4000UL)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400UL)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800UL)
#define UART4_BASE            (APB1PERIPH_BASE + 0x4C00UL)
#define UART5_BASE            (APB1PERIPH_BASE + 0x5000UL)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00UL)
#define FMPI2C1_BASE          (APB1PERIPH_BASE + 0x6000UL)
#define CAN1_BASE             (APB1PERIPH_BASE + 0x6400UL)
#define CAN2_BASE             (APB1PERIPH_BASE + 0x6800UL)
#define CEC_BASE              (APB1PERIPH_BASE + 0x6C00UL)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000UL)
#define DAC_BASE              (APB1PERIPH_BASE + 0x7400UL)

//ABP2 Peripheral definitions
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000UL)
#define TIM8_BASE             (APB2PERIPH_BASE + 0x0400UL)
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000UL)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400UL)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2000UL)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2100UL)
#define ADC3_BASE             (APB2PERIPH_BASE + 0x2200UL)
#define ADC123_COMMON_BASE    (APB2PERIPH_BASE + 0x2300UL)
#define SDIMMC_BASE           (APB2PERIPH_BASE + 0x2C00UL)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000UL)
#define SPI4_BASE             (APB2PERIPH_BASE + 0x3400UL)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800UL)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00UL)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000UL)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400UL)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800UL)
#define SAI1_BASE             (APB2PERIPH_BASE + 0x5800UL)
#define SAI2_BASE             (APB2PERIPH_BASE + 0x5C00UL)

//AHB1 Peripheral definitions
#define GPIOA_BASE              (AHB1PERIPH_BASE + 0x00000UL)
#define GPIOB_BASE              (AHB1PERIPH_BASE + 0x00400UL)
#define GPIOC_BASE              (AHB1PERIPH_BASE + 0x00800UL)
#define GPIOD_BASE              (AHB1PERIPH_BASE + 0x00C00UL)
#define GPIOE_BASE              (AHB1PERIPH_BASE + 0x01000UL)
#define GPIOF_BASE              (AHB1PERIPH_BASE + 0x01400UL)
#define GPIOG_BASE              (AHB1PERIPH_BASE + 0x01800UL)
#define GPIOH_BASE              (AHB1PERIPH_BASE + 0x01C00UL)
#define CRC_BASE                (AHB1PERIPH_BASE + 0x03000UL)
#define RCC_BASE                (AHB1PERIPH_BASE + 0x03800UL)
#define FLASH_R_BASE            (AHB1PERIPH_BASE + 0x03C00UL)
#define DMA1_BASE               (AHB1PERIPH_BASE + 0x06000UL)
#define DMA2_BASE               (AHB1PERIPH_BASE + 0x06400UL)
#define USB_OTG_HS_BASE         (AHB1PERIPH_BASE + 0x20000UL)

//AHB2 Peripheral definitions
#define USB_OTG_FS_BASE         (AHB2PERIPH_BASE)
#define DCMI_BASE               (AHB2PERIPH_BASE + 0x50000UL)


//AHB3 Peripheral definitions
#define FMC_R_BASE            AHB3PERIPH_BASE + 0x40000000UL /*!< FMC registers base address                                                 */
#define QSPI_R_BASE           AHB3PERIPH_BASE + 0x40001000UL /*!< QuadSPI registers base address                                             */



/**************************************Peripheral Registers Definitions ************************************************/


typedef struct
{
    volatile uint32_t RCC_CR;        //RCC clock control register. Controls clocks for each of the registers
    volatile uint32_t RCC_PLLCFGR;   //PLL configuration register. controls different plls
    volatile uint32_t RCC_CFGR;      //RCC clock configuration register, selects clock source
    volatile uint32_t RCC_CIR;       //RCC clock interrupt register
    volatile uint32_t RCC_AHB1RSTR;  // RCC AHB1 peripheral reset register
    volatile uint32_t RCC_AHB2RSTR;  // RCC AHB2 peripheral reset register
    volatile uint32_t RCC_AHB3RSTR;  // RCC AHB3 peripheral reset register
    volatile uint32_t RESERVED0;
    volatile uint32_t APB1RSTR;      // RCC APB1 peripheral reset register
    volatile uint32_t APB2RSTR;      // RCC APB2 peripheral reset register
    volatile uint32_t RESERVED1[2];  // Reserved, 0x28-0x2C
    volatile uint32_t AHB1ENR;       // RCC AHB1 peripheral clock register
    volatile uint32_t AHB2ENR;       // RCC AHB2 peripheral clock register
    volatile uint32_t AHB3ENR;       // RCC AHB3 peripheral clock register
    volatile uint32_t RESERVED2;     // Reserved, 0x3C
    volatile uint32_t APB1ENR;       // RCC APB1 peripheral clock enable register
    volatile uint32_t APB2ENR;       // RCC APB2 peripheral clock enable register
    volatile uint32_t RESERVED3[2];  // Reserved, 0x48-0x4C
    volatile uint32_t AHB1LPENR;     // RCC AHB1 peripheral clock enable in low power mode register
    volatile uint32_t AHB2LPENR;     // RCC AHB2 peripheral clock enable in low power mode register
    volatile uint32_t AHB3LPENR;     // RCC AHB3 peripheral clock enable in low power mode register
    volatile uint32_t RESERVED4;     // Reserved, 0x5C
    volatile uint32_t APB1LPENR;     // RCC APB1 peripheral clock enable in low power mode register
    volatile uint32_t APB2LPENR;     // RCC APB2 peripheral clock enable in low power mode register
    volatile uint32_t RESERVED5[2];  // Reserved, 0x68-0x6C
    volatile uint32_t BDCR;          // RCC Backup domain control register
    volatile uint32_t CSR;           // RCC clock control & status register
    volatile uint32_t RESERVED6[2];  // Reserved, 0x78-0x7C
    volatile uint32_t SSCGR;         // RCC spread spectrum clock generation register
    volatile uint32_t PLLI2SCFGR;    // RCC PLLI2S configuration register
    volatile uint32_t PLLSAICFGR;    // RCC PLLSAI configuration register
    volatile uint32_t DCKCFGR;       // RCC Dedicated Clocks configuration register
    volatile uint32_t CKGATENR;      // RCC Clocks Gated ENable Register
    volatile uint32_t DCKCFGR2;      // RCC Dedicated Clocks configuration register 2
}RCC_Register_Definition_Type;


typedef struct
{
    volatile uint32_t GPIOx_MODER;   //These bits are written by software to configure the I/O direction mode
    volatile uint32_t GPIOx_OTYPER;  //These bits are written by software to configure the output type of the I/O port
    volatile uint32_t GPIOx_OSPEEDR; //These bits are written by software to configure the I/O output speed.
    volatile uint32_t GPIOx_PUPDR;   //These bits are written by software to configure the I/O pull-up or pull-down
    volatile uint32_t GPIOx_IDR;     // These bits are read-only and can be accessed in word mode only. They contain the input
                                     // value of the corresponding I/O port
    volatile uint32_t GPIOx_ODR;     //These bits can be read and written by software
    volatile uint32_t GPIOx_BSRR;    //These bits are write-only and can be accessed in word, half-word or byte mode. A read to
                                     //these bits returns the value 0x0000.
                                     //0: No action on the corresponding ODRx bit
                                     //1: Resets the corresponding ODRx bit
    volatile uint32_t GPIOx_LCKR;    //This register is used to lock the configuration of the port bits when a correct write sequence
                                     //is applied to bit 16 (LCKK).
    volatile uint32_t GPIOx_AFRL;    // Alternate function selection for port x bit y (y = 0..7)
    volatile uint32_t GPIOx_AFRH;    // Alternate function selection for port x bit y (y = 8..15)
} GPIO_Register_Definition_Type;


typedef struct
{
    volatile uint32_t SYSCFG_MEMRMP;        //0x00
    volatile uint32_t SYSCFG_PMC;           //0x04
    volatile uint32_t SYSCFG_EXTICRx[4];    //0x08 - 0x17
    volatile uint32_t UNUSED01[2];          //0x18 - 0x1F
    volatile uint32_t CMPCR;                //0x20
    volatile uint32_t UNUSED02[2];          //0x24 - 0x2B
    volatile uint32_t CFGR;                 //0x2C

}SYSCFG_Register_Definition_Type;


typedef struct
{
    volatile uint32_t EXTI_IMR;
    volatile uint32_t EXTI_EMR;
    volatile uint32_t EXTI_RTSR;
    volatile uint32_t EXTI_FTSR;
    volatile uint32_t EXTI_SWIER;
    volatile uint32_t EXTI_PR;
}EXTI_Register_Definition_Type;

typedef enum
{
    /******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
    NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
    MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
    BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
    UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
    SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
    DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
    PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
    SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
    /******  STM32 specific Interrupt Numbers **********************************************************************/
    WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
    PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
    TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
    RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
    FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
    RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
    EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
    EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
    EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
    EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
    EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
    DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
    DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
    DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
    DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
    DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
    DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
    DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
    ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
    CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
    CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
    CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
    CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
    EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
    TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
    TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
    TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
    TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
    TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
    TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
    TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
    I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
    I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
    I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
    I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
    SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
    SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
    USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
    USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
    USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
    EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
    RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
    OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
    TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
    TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
    TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
    TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare global interrupt                             */
    DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
    FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
    SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
    TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
    SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
    UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
    UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
    TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
    TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
    DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
    DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
    DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
    DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
    DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
    CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
    CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
    CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
    CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
    OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
    DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
    DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
    DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
    USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
    I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
    I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
    OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
    OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
    OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
    OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
    DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
    FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
    SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
    SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
    SAI2_IRQn                   = 91,     /*!< SAI2 global Interrupt                                             */
    QUADSPI_IRQn                = 92,     /*!< QuadSPI global Interrupt                                          */
    CEC_IRQn                    = 93,     /*!< CEC global Interrupt                                              */
    SPDIF_RX_IRQn               = 94,     /*!< SPDIF-RX global Interrupt                                          */
    FMPI2C1_EV_IRQn             = 95,     /*!< FMPI2C1 Event Interrupt                                           */
    FMPI2C1_ER_IRQn             = 96      /*!< FMPI2C1 Error Interrupt                                           */
} IRQn_Type;

typedef enum
{
    NVIC_IRC_PRIO_0 = 0,
    NVIC_IRC_PRIO_1 = 1,
    NVIC_IRC_PRIO_2 = 2,
    NVIC_IRC_PRIO_3 = 3,
    NVIC_IRC_PRIO_4 = 4,
    NVIC_IRC_PRIO_5 = 5,
    NVIC_IRC_PRIO_6 = 6,
    NVIC_IRC_PRIO_7 = 7,
    NVIC_IRC_PRIO_8 = 8,
    NVIC_IRC_PRIO_9 = 9,
    NVIC_IRC_PRIO_10 = 10,
    NVIC_IRC_PRIO_11 = 11,
    NVIC_IRC_PRIO_12 = 12,
    NVIC_IRC_PRIO_13 = 13,
    NVIC_IRC_PRIO_14 = 14,
    NVIC_IRC_PRIO_15 = 15,
}NVIC_Priority;

/**************************************Peripheral  Definitions ************************************************/

#define RCC   (( RCC_Register_Definition_Type *) RCC_BASE)

#define GPIOA  (( GPIO_Register_Definition_Type *) GPIOA_BASE)
#define GPIOB  (( GPIO_Register_Definition_Type *) GPIOB_BASE)
#define GPIOC  (( GPIO_Register_Definition_Type *) GPIOC_BASE)
#define GPIOD  (( GPIO_Register_Definition_Type *) GPIOD_BASE)
#define GPIOE  (( GPIO_Register_Definition_Type *) GPIOE_BASE)
#define GPIOF  (( GPIO_Register_Definition_Type *) GPIOF_BASE)
#define GPIOG  (( GPIO_Register_Definition_Type *) GPIOG_BASE)
#define GPIOH  (( GPIO_Register_Definition_Type *) GPIOH_BASE)

#define SYSCFG (( SYSCFG_Register_Definition_Type *) SYSCFG_BASE)

#define EXTI   (( EXTI_Register_Definition_Type *) EXTI_BASE)



/*******************  Bit definition for RCC register  ********************/
//Clock enable peripheral macros
#define RCC_CR_HSION               (RCC->AHB1ENR |= (0x01 <<  0)) //HSI oscillator ON
#define RCC_CR_HSIRDY              (RCC->AHB1ENR |= (0x01 <<  1)) //HSIRDY


#define RCC_CR_HSEON               (RCC->AHB1ENR |= (0x01 << 16)) //HSE oscillator ON
#define RCC_CR_HSERDY              (RCC->AHB1ENR |= (0x01 << 17)) //HSERDY



#define RCC_AHB1ENR_GPIOA_SET       (RCC->AHB1ENR |= (0x01 <<  0)) //GPIO port A clock enabled
#define RCC_AHB1ENR_GPIOB_SET       (RCC->AHB1ENR |= (0x01 <<  1)) //GPIO port B clock enabled
#define RCC_AHB1ENR_GPIOC_SET       (RCC->AHB1ENR |= (0x01 <<  2)) //GPIO port C clock enabled
#define RCC_AHB1ENR_GPIOD_SET       (RCC->AHB1ENR |= (0x01 <<  3)) //GPIO port D clock enabled
#define RCC_AHB1ENR_GPIOE_SET       (RCC->AHB1ENR |= (0x01 <<  4)) //GPIO port E clock enabled
#define RCC_AHB1ENR_GPIOF_SET       (RCC->AHB1ENR |= (0x01 <<  5)) //GPIO port F clock enabled
#define RCC_AHB1ENR_GPIOG_SET       (RCC->AHB1ENR |= (0x01 <<  6)) //GPIO port G clock enabled
#define RCC_AHB1ENR_GPIOH_SET       (RCC->AHB1ENR |= (0x01 <<  7)) //GPIO port H clock enabled
#define RCC_AHB1ENR_CRC_SET         (RCC->AHB1ENR |= (0x01 << 12)) //CRC clock enabled
#define RCC_AHB1ENR_DMA1_SET        (RCC->AHB1ENR |= (0x01 << 21)) //DMA1 clock enable
#define RCC_AHB1ENR_DMA2_SET        (RCC->AHB1ENR |= (0x01 << 22)) //DMA2 clock enable
#define RCC_AHB1ENR_OTGHS_SET       (RCC->AHB1ENR |= (0x01 << 29)) //USB OTG HS clock enable
#define RCC_AHB1ENR_OTGHSULPI_SET   (RCC->AHB1ENR |= (0x01 << 30)) //USB OTG HS clock enable

#define RCC_AHB1ENR_GPIOA_CLR       (RCC->AHB1ENR |= (0x01 <<  0)) //GPIO port A clock enabled
#define RCC_AHB1ENR_GPIOB_CLR       (RCC->AHB1ENR |= (0x01 <<  1)) //GPIO port B clock enabled
#define RCC_AHB1ENR_GPIOC_CLR       (RCC->AHB1ENR |= (0x01 <<  2)) //GPIO port C clock enabled
#define RCC_AHB1ENR_GPIOD_CLR       (RCC->AHB1ENR |= (0x01 <<  3)) //GPIO port D clock enabled
#define RCC_AHB1ENR_GPIOE_CLR       (RCC->AHB1ENR |= (0x01 <<  4)) //GPIO port E clock enabled
#define RCC_AHB1ENR_GPIOF_CLR       (RCC->AHB1ENR |= (0x01 <<  5)) //GPIO port F clock enabled
#define RCC_AHB1ENR_GPIOG_CLR       (RCC->AHB1ENR |= (0x01 <<  6)) //GPIO port G clock enabled
#define RCC_AHB1ENR_GPIOH_CLR       (RCC->AHB1ENR |= (0x01 <<  7)) //GPIO port H clock enabled
#define RCC_AHB1ENR_CRC_CLR         (RCC->AHB1ENR |= (0x01 << 12)) //CRC clock enabled
#define RCC_AHB1ENR_DMA1_CLR        (RCC->AHB1ENR |= (0x01 << 21)) //DMA1 clock enable
#define RCC_AHB1ENR_DMA2_CLR        (RCC->AHB1ENR |= (0x01 << 22)) //DMA2 clock enable
#define RCC_AHB1ENR_OTGHS_CLR       (RCC->AHB1ENR |= (0x01 << 29)) //USB OTG HS clock enable
#define RCC_AHB1ENR_OTGHSULPI_CLR   (RCC->AHB1ENR |= (0x01 << 30)) //USB OTG HS clock enable


#define RCC_AHB1RSTR_GPIOA_RST_SET       (RCC->AHB1ENR |= (0x01 <<  0)) //GPIO port A clock enabled
#define RCC_AHB1RSTR_GPIOB_RST_SET       (RCC->AHB1ENR |= (0x01 <<  1)) //GPIO port B clock enabled
#define RCC_AHB1RSTR_GPIOC_RST_SET       (RCC->AHB1ENR |= (0x01 <<  2)) //GPIO port C clock enabled
#define RCC_AHB1RSTR_GPIOD_RST_SET       (RCC->AHB1ENR |= (0x01 <<  3)) //GPIO port D clock enabled
#define RCC_AHB1RSTR_GPIOE_RST_SET       (RCC->AHB1ENR |= (0x01 <<  4)) //GPIO port E clock enabled
#define RCC_AHB1RSTR_GPIOF_RST_SET       (RCC->AHB1ENR |= (0x01 <<  5)) //GPIO port F clock enabled
#define RCC_AHB1RSTR_GPIOG_RST_SET       (RCC->AHB1ENR |= (0x01 <<  6)) //GPIO port G clock enabled
#define RCC_AHB1RSTR_GPIOH_RST_SET       (RCC->AHB1ENR |= (0x01 <<  7)) //GPIO port H clock enabled
#define RCC_AHB1RSTR_CRC_RST_SET         (RCC->AHB1ENR |= (0x01 << 12)) //CRC clock enabled
#define RCC_AHB1RSTR_DMA1_RST_SET        (RCC->AHB1ENR |= (0x01 << 21)) //DMA1 clock enable
#define RCC_AHB1RSTR_DMA2_RST_SET        (RCC->AHB1ENR |= (0x01 << 22)) //DMA2 clock enable
#define RCC_AHB1RSTR_OTGHS_RST_SET       (RCC->AHB1ENR |= (0x01 << 29)) //USB OTG HS clock enable

#define RCC_AHB1RSTR_GPIOA_RST_CLR       (RCC->AHB1ENR &= ~(0x01 <<  0)) //GPIO port A clock enabled
#define RCC_AHB1RSTR_GPIOB_RST_CLR       (RCC->AHB1ENR &= ~(0x01 <<  1)) //GPIO port B clock enabled
#define RCC_AHB1RSTR_GPIOC_RST_CLR       (RCC->AHB1ENR &= ~(0x01 <<  2)) //GPIO port C clock enabled
#define RCC_AHB1RSTR_GPIOD_RST_CLR       (RCC->AHB1ENR &= ~(0x01 <<  3)) //GPIO port D clock enabled
#define RCC_AHB1RSTR_GPIOE_RST_CLR       (RCC->AHB1ENR &= ~(0x01 <<  4)) //GPIO port E clock enabled
#define RCC_AHB1RSTR_GPIOF_RST_CLR       (RCC->AHB1ENR &= ~(0x01 <<  5)) //GPIO port F clock enabled
#define RCC_AHB1RSTR_GPIOG_RST_CLR       (RCC->AHB1ENR &= ~(0x01 <<  6)) //GPIO port G clock enabled
#define RCC_AHB1RSTR_GPIOH_RST_CLR       (RCC->AHB1ENR &= ~(0x01 <<  7)) //GPIO port H clock enabled
#define RCC_AHB1RSTR_CRC_RST_CLR         (RCC->AHB1ENR &= ~(0x01 << 12)) //CRC clock enabled
#define RCC_AHB1RSTR_DMA1_RST_CLR        (RCC->AHB1ENR &= ~(0x01 << 21)) //DMA1 clock enable
#define RCC_AHB1RSTR_DMA2_RST_CLR        (RCC->AHB1ENR &= ~(0x01 << 22)) //DMA2 clock enable
#define RCC_AHB1RSTR_OTGHS_RST_CLR       (RCC->AHB1ENR &= ~(0x01 << 29)) //USB OTG HS clock enable

#define RCC_APB2ENR_TIM1_SET             (RCC->APB2ENR |= (0x01 <<  0))
#define RCC_APB2ENR_TIM8_SET             (RCC->APB2ENR |= (0x01 <<  1))
#define RCC_APB2ENR_USART1_SET           (RCC->APB2ENR |= (0x01 <<  4))
#define RCC_APB2ENR_USART6_SET           (RCC->APB2ENR |= (0x01 <<  5))
#define RCC_APB2ENR_ADC1_SET             (RCC->APB2ENR |= (0x01 <<  8))
#define RCC_APB2ENR_ADC2_SET             (RCC->APB2ENR |= (0x01 <<  9))
#define RCC_APB2ENR_ADC3_SET             (RCC->APB2ENR |= (0x01 <<  10))
#define RCC_APB2ENR_SDIO_SET             (RCC->APB2ENR |= (0x01 <<  11))
#define RCC_APB2ENR_SPI1_SET             (RCC->APB2ENR |= (0x01 <<  12))
#define RCC_APB2ENR_SPI4_SET             (RCC->APB2ENR |= (0x01 <<  13))
#define RCC_APB2ENR_SYSCFG_SET           (RCC->APB2ENR |= (0x01 <<  14))
#define RCC_APB2ENR_TIM9_SET             (RCC->APB2ENR |= (0x01 <<  16))
#define RCC_APB2ENR_TIM10_SET            (RCC->APB2ENR |= (0x01 <<  17))
#define RCC_APB2ENR_TIM11_SET            (RCC->APB2ENR |= (0x01 <<  18))
#define RCC_APB2ENR_SAI1_SET             (RCC->APB2ENR |= (0x01 <<  22))
#define RCC_APB2ENR_SAI2_SET             (RCC->APB2ENR |= (0x01 <<  23))

#define RCC_APB2ENR_TIM1_CLR             (RCC->APB2ENR |= (0x01 <<  0))
#define RCC_APB2ENR_TIM8_CLR             (RCC->APB2ENR |= (0x01 <<  1))
#define RCC_APB2ENR_USART1_CLR           (RCC->APB2ENR |= (0x01 <<  4))
#define RCC_APB2ENR_USART6_CLR           (RCC->APB2ENR |= (0x01 <<  5))
#define RCC_APB2ENR_ADC1_CLR             (RCC->APB2ENR |= (0x01 <<  8))
#define RCC_APB2ENR_ADC2_CLR             (RCC->APB2ENR |= (0x01 <<  9))
#define RCC_APB2ENR_ADC3_CLR             (RCC->APB2ENR |= (0x01 <<  10))
#define RCC_APB2ENR_SDIO_CLR             (RCC->APB2ENR |= (0x01 <<  11))
#define RCC_APB2ENR_SPI1_CLR             (RCC->APB2ENR |= (0x01 <<  12))
#define RCC_APB2ENR_SPI4_CLR             (RCC->APB2ENR |= (0x01 <<  13))
#define RCC_APB2ENR_SYSCFG_CLR           (RCC->APB2ENR |= (0x01 <<  14))
#define RCC_APB2ENR_TIM9_CLR             (RCC->APB2ENR |= (0x01 <<  16))
#define RCC_APB2ENR_TIM10_CLR            (RCC->APB2ENR |= (0x01 <<  17))
#define RCC_APB2ENR_TIM11_CLR            (RCC->APB2ENR |= (0x01 <<  18))
#define RCC_APB2ENR_SAI1_CLR             (RCC->APB2ENR |= (0x01 <<  22))
#define RCC_APB2ENR_SAI2_CLR             (RCC->APB2ENR |= (0x01 <<  23))


/**************************** General Macro Definitions ********************************/

static inline uint8_t gpio_to_port_number (GPIO_Register_Definition_Type * GPIOx)
{
    uint8_t  retValue = 0;

    if(GPIOx == GPIOA) retValue = 0;
    else if (GPIOx == GPIOB) retValue = 1;
    else if (GPIOx == GPIOC) retValue = 2;
    else if (GPIOx == GPIOD) retValue = 3;
    else if (GPIOx == GPIOE) retValue = 4;
    else if (GPIOx == GPIOF) retValue = 5;
    else if (GPIOx == GPIOG) retValue = 6;
    else if (GPIOx == GPIOH) retValue = 7;
    else  retValue = 0;

    return retValue;
}



#endif /* INCLUDE_STM32F446XX_H_ */
