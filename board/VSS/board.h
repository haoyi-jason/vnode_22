/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

#ifndef BOARD_H
#define BOARD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Setup for STMicroelectronics STM32F4-Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32F4_DISCOVERY
#define BOARD_NAME                  "SensorHub V2"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
//#define STM32F411xE
#define STM32F4xx
#define STM32F411xx
/*
 * IO pins assignments.
 */
#define GPIOA_UART2_CTS      0U
#define GPIOA_UART2_RTS      1U
#define GPIOA_UART2_TX       2U
#define GPIOA_UART2_RX       3U
#define GPIOA_SPI1_CS        4U
#define GPIOA_SPI1_SCK       5U
#define GPIOA_SPI1_MISO      6U
#define GPIOA_SPI1_MOSI      7U
#define GPIOA_SPI2_CS       8U
#define GPIOA_UART1_TX       9U
#define GPIOA_UART1_RX       10U
#define GPIOA_BMI_INT1       11U
#define GPIOA_BMI_INT2         12U
#define GPIOA_SWCLK          13U
#define GPIOA_SWDIO          14U
#define GPIOA_BMI_DRDY         15U

#define GPIOB_0              0U
#define GPIOB_1              1U
#define GPIOB_2        2U
#define GPIOB_SPI3_SCK       3U
#define GPIOB_SPI3_MISO      4U
#define GPIOB_SPI3_MOSI      5U
#define GPIOB_I2C1_SCL       6U
#define GPIOB_I2C1_SDA       7U
#define GPIOB_ADXL_INT2       8U
#define GPIOB_ADXL_INT1          9U
#define GPIOB_SD_CD      10U
#define GPIOB_I2C2_SDA      11U
#define GPIOB_FS_CS0       12U
#define GPIOB_SPI2_SCK      13U
#define GPIOB_SPI2_MISO     14U
#define GPIOB_SPI2_MOSI     15U
                            
#define GPIOC_AD0    0U
#define GPIOC_AD1   1U
#define GPIOC_AD2    2U
#define GPIOC_LOG_LED    3U
#define GPIOC_4    4U
#define GPIOC_5    5U
#define GPIOC_FS_CS1    6U
#define GPIOC_FS_CS2    7U
#define GPIOC_SDIO_D0    8U
#define GPIOC_SDIO_D1    9U
#define GPIOC_SDIO_D2   10U
#define GPIOC_SDIO_D3   11U
#define GPIOC_SDIO_CLK   12U
#define GPIOC_USER_BTN   13U
#define GPIOC_OSC32_IN    14U
#define GPIOC_OSC32_OUT   15U

#define GPIOD_0    0U
#define GPIOD_1   1U
#define GPIOD_SDIO_CMD    2U
#define GPIOD_3    3U
#define GPIOD_4    4U
#define GPIOD_5    5U
#define GPIOD_6    6U
#define GPIOD_7    7U
#define GPIOD_8    8U
#define GPIOD_9    9U
#define GPIOD_10    10U
#define GPIOD_11    11U
#define GPIOD_12    12U
#define GPIOD_13    13U
#define GPIOD_14    14U
#define GPIOD_15    15U

#define GPIOE_0   0U
#define GPIOE_1   1U
#define GPIOE_2   2U
#define GPIOE_3   3U
#define GPIOE_4   4U
#define GPIOE_5   5U
#define GPIOE_6   6U
#define GPIOE_7   7U
#define GPIOE_SW11   8U
#define GPIOE_SW12   9U
#define GPIOE_SW13   10U
#define GPIOE_SW14   11U
#define GPIOE_SW21   12U
#define GPIOE_SW22   13U
#define GPIOE_SW23   14U
#define GPIOE_SW24   15U

#define GPIOF_PIN0                  0U
#define GPIOF_PIN1                  1U
#define GPIOF_PIN2                  2U
#define GPIOF_PIN3                  3U
#define GPIOF_PIN4                  4U
#define GPIOF_PIN5                  5U
#define GPIOF_PIN6                  6U
#define GPIOF_PIN7                  7U
#define GPIOF_PIN8                  8U
#define GPIOF_PIN9                  9U
#define GPIOF_PIN10                 10U
#define GPIOF_PIN11                 11U
#define GPIOF_PIN12                 12U
#define GPIOF_PIN13                 13U
#define GPIOF_PIN14                 14U
#define GPIOF_PIN15                 15U

#define GPIOG_PIN0                  0U
#define GPIOG_PIN1                  1U
#define GPIOG_PIN2                  2U
#define GPIOG_PIN3                  3U
#define GPIOG_PIN4                  4U
#define GPIOG_PIN5                  5U
#define GPIOG_PIN6                  6U
#define GPIOG_PIN7                  7U
#define GPIOG_PIN8                  8U
#define GPIOG_PIN9                  9U
#define GPIOG_PIN10                 10U
#define GPIOG_PIN11                 11U
#define GPIOG_PIN12                 12U
#define GPIOG_PIN13                 13U
#define GPIOG_PIN14                 14U
#define GPIOG_PIN15                 15U

#define GPIOH_OSC_IN                0U
#define GPIOH_OSC_OUT               1U
#define GPIOH_PIN2                  2U
#define GPIOH_PIN3                  3U
#define GPIOH_PIN4                  4U
#define GPIOH_PIN5                  5U
#define GPIOH_PIN6                  6U
#define GPIOH_PIN7                  7U
#define GPIOH_PIN8                  8U
#define GPIOH_PIN9                  9U
#define GPIOH_PIN10                 10U
#define GPIOH_PIN11                 11U
#define GPIOH_PIN12                 12U
#define GPIOH_PIN13                 13U
#define GPIOH_PIN14                 14U
#define GPIOH_PIN15                 15U

#define GPIOI_PIN0                  0U
#define GPIOI_PIN1                  1U
#define GPIOI_PIN2                  2U
#define GPIOI_PIN3                  3U
#define GPIOI_PIN4                  4U
#define GPIOI_PIN5                  5U
#define GPIOI_PIN6                  6U
#define GPIOI_PIN7                  7U
#define GPIOI_PIN8                  8U
#define GPIOI_PIN9                  9U
#define GPIOI_PIN10                 10U
#define GPIOI_PIN11                 11U
#define GPIOI_PIN12                 12U
#define GPIOI_PIN13                 13U
#define GPIOI_PIN14                 14U
#define GPIOI_PIN15                 15U

/*
 * IO lines assignments.
 */
#define LINE_BUTTON                 PAL_LINE(GPIOA, 0U)
#define LINE_LRCK                   PAL_LINE(GPIOA, 4U)
#define LINE_SPC                    PAL_LINE(GPIOA, 5U)
#define LINE_SDO                    PAL_LINE(GPIOA, 6U)
#define LINE_SDI                    PAL_LINE(GPIOA, 7U)
#define LINE_VBUS_FS                PAL_LINE(GPIOA, 9U)
#define LINE_OTG_FS_ID              PAL_LINE(GPIOA, 10U)
#define LINE_OTG_FS_DM              PAL_LINE(GPIOA, 11U)
#define LINE_OTG_FS_DP              PAL_LINE(GPIOA, 12U)
#define LINE_SWDIO                  PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK                  PAL_LINE(GPIOA, 14U)
#define LINE_SWO                    PAL_LINE(GPIOB, 3U)
#define LINE_SCL                    PAL_LINE(GPIOB, 6U)
#define LINE_SDA                    PAL_LINE(GPIOB, 9U)
#define LINE_CLK_IN                 PAL_LINE(GPIOB, 10U)
#define LINE_OTG_FS_POWER_ON        PAL_LINE(GPIOC, 0U)
#define LINE_PDM_OUT                PAL_LINE(GPIOC, 3U)
#define LINE_MCLK                   PAL_LINE(GPIOC, 7U)
#define LINE_SCLK                   PAL_LINE(GPIOC, 10U)
#define LINE_SDIN                   PAL_LINE(GPIOC, 12U)
#define LINE_RESET                  PAL_LINE(GPIOD, 4U)
#define LINE_OVER_CURRENT           PAL_LINE(GPIOD, 5U)
#define LINE_LED4                   PAL_LINE(GPIOD, 12U)
#define LINE_LED3                   PAL_LINE(GPIOD, 13U)
#define LINE_LED5                   PAL_LINE(GPIOD, 14U)
#define LINE_LED6                   PAL_LINE(GPIOD, 15U)
#define LINE_INT1                   PAL_LINE(GPIOE, 0U)
#define LINE_INT2                   PAL_LINE(GPIOE, 1U)
#define LINE_CS_SPI                 PAL_LINE(GPIOE, 3U)
#define LINE_OSC_IN                 PAL_LINE(GPIOH, 0U)
#define LINE_OSC_OUT                PAL_LINE(GPIOH, 1U)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
 *
 * PA0  - BUTTON                    (input floating).
 * PA1  - PIN1                      (input pullup).
 * PA2  - PIN2                      (input pullup).
 * PA3  - PIN3                      (input pullup).
 * PA4  - LRCK                      (alternate 6).
 * PA5  - SPC                       (alternate 5).
 * PA6  - SDO                       (alternate 5).
 * PA7  - SDI                       (alternate 5).
 * PA8  - PIN8                      (input pullup).
 * PA9  - VBUS_FS                   (input floating).
 * PA10 - OTG_FS_ID                 (alternate 10).
 * PA11 - OTG_FS_DM                 (alternate 10).
 * PA12 - OTG_FS_DP                 (alternate 10).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - PIN15                     (input pullup).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_OUTPUT(GPIOA_UART2_CTS)         |\
                                     PIN_MODE_OUTPUT(GPIOA_UART2_RTS)         |\
                                     PIN_MODE_INPUT(GPIOA_UART2_TX )                  |\
                                     PIN_MODE_OUTPUT(GPIOA_UART2_RX )                    |\
                                        PIN_MODE_OUTPUT(GPIOA_SPI1_CS  )             |\
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_SCK )         |\
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MISO)        |\
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MOSI)        |\
                                         PIN_MODE_OUTPUT(GPIOA_SPI2_CS )              |\
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_TX )         |\
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_RX )         |\
                                         PIN_MODE_INPUT(GPIOA_BMI_INT1   )                   |\
                                         PIN_MODE_INPUT(GPIOA_BMI_INT2   )                   |\
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO    )            |\
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK    )            |\
                                        PIN_MODE_OUTPUT(GPIOA_BMI_DRDY   ))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_UART2_CTS)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART2_RTS)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART2_TX )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART2_RX )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_CS  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_SCK )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MISO)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MOSI)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI2_CS )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_TX )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_RX )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_BMI_INT1   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_BMI_INT2   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO    )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK    )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_BMI_DRDY   ))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_UART2_CTS)           |\
                                     PIN_OSPEED_HIGH(GPIOA_UART2_RTS)           |\
                                     PIN_OSPEED_HIGH(GPIOA_UART2_TX )           |\
                                     PIN_OSPEED_HIGH(GPIOA_UART2_RX )           |\
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_CS  )           |\
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_SCK )            |\
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_MISO)           |\
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_MOSI)           |\
                                     PIN_OSPEED_HIGH(GPIOA_SPI2_CS )           |\
                                     PIN_OSPEED_HIGH(GPIOA_UART1_TX )           |\
                                     PIN_OSPEED_HIGH(GPIOA_UART1_RX )           |\
                                     PIN_OSPEED_HIGH(GPIOA_BMI_INT1   )               |\
                                     PIN_OSPEED_HIGH(GPIOA_BMI_INT2   )           |\
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO    )           |\
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK    )           |\
                                     PIN_OSPEED_HIGH(GPIOA_BMI_DRDY   ))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIOA_UART2_CTS)         |\
                                     PIN_PUPDR_PULLUP(GPIOA_UART2_RTS)         |\
                                     PIN_PUPDR_PULLUP(GPIOA_UART2_TX )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_UART2_RX )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_CS  )           |\
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_SCK )            |\
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_MISO)           |\
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_MOSI)         |\
                                     PIN_PUPDR_PULLUP(GPIOA_SPI2_CS )           |\
                                     PIN_PUPDR_FLOATING(GPIOA_UART1_TX )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_UART1_RX )         |\
                                     PIN_PUPDR_FLOATING(GPIOA_BMI_INT1   )             |\
                                     PIN_PUPDR_FLOATING(GPIOA_BMI_INT2   )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO    )         |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK    )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_BMI_DRDY   ))
#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_UART2_CTS)               |\
                                     PIN_ODR_LOW(GPIOA_UART2_RTS)               |\
                                     PIN_ODR_HIGH(GPIOA_UART2_TX )               |\
                                     PIN_ODR_HIGH(GPIOA_UART2_RX )               |\
                                     PIN_ODR_HIGH(GPIOA_SPI1_CS  )               |\
                                     PIN_ODR_HIGH(GPIOA_SPI1_SCK )                |\
                                     PIN_ODR_HIGH(GPIOA_SPI1_MISO)               |\
                                     PIN_ODR_HIGH(GPIOA_SPI1_MOSI)               |\
                                     PIN_ODR_HIGH(GPIOA_SPI2_CS )               |\
                                     PIN_ODR_HIGH(GPIOA_UART1_TX )               |\
                                     PIN_ODR_HIGH(GPIOA_UART1_RX )               |\
                                     PIN_ODR_HIGH(GPIOA_BMI_INT1   )                   |\
                                     PIN_ODR_HIGH(GPIOA_BMI_INT2   )               |\
                                     PIN_ODR_HIGH(GPIOA_SWDIO    )               |\
                                     PIN_ODR_HIGH(GPIOA_SWCLK    )               |\
                                     PIN_ODR_HIGH(GPIOA_BMI_DRDY   ))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_UART2_CTS,0)            |\
                                     PIN_AFIO_AF(GPIOA_UART2_RTS,0)            |\
                                     PIN_AFIO_AF(GPIOA_UART2_TX ,0)            |\
                                     PIN_AFIO_AF(GPIOA_UART2_RX ,0)            |\
                                     PIN_AFIO_AF(GPIOA_SPI1_CS  ,0)            |\
                                     PIN_AFIO_AF(GPIOA_SPI1_SCK ,5)             |\
                                     PIN_AFIO_AF(GPIOA_SPI1_MISO,5)            |\
                                     PIN_AFIO_AF(GPIOA_SPI1_MOSI,5))            
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_SPI1_CS ,0)             |\
                                     PIN_AFIO_AF(GPIOA_UART1_TX ,7)             |\
                                     PIN_AFIO_AF(GPIOA_UART1_RX ,7)             |\
                                     PIN_AFIO_AF(GPIOA_BMI_INT1   ,0)                 |\
                                     PIN_AFIO_AF(GPIOA_BMI_INT2   ,0)             |\
                                     PIN_AFIO_AF(GPIOA_SWDIO    ,0)             |\
                                     PIN_AFIO_AF(GPIOA_SWCLK    ,0)             |\
                                     PIN_AFIO_AF(GPIOA_BMI_DRDY   ,0))

/*
 * GPIOB setup:
 *
 * PB0  - PIN0                      (input pullup).
 * PB1  - PIN1                      (input pullup).
 * PB2  - PIN2                      (input pullup).
 * PB3  - SWO                       (alternate 0).
 * PB4  - PIN4                      (input pullup).
 * PB5  - PIN5                      (input pullup).
 * PB6  - SCL                       (alternate 4).
 * PB7  - PIN7                      (input pullup).
 * PB8  - PIN8                      (input pullup).
 * PB9  - SDA                       (alternate 4).
 * PB10 - CLK_IN                    (input pullup).
 * PB11 - PIN11                     (input pullup).
 * PB12 - PIN12                     (input pullup).
 * PB13 - PIN13                     (input pullup).
 * PB14 - PIN14                     (input pullup).
 * PB15 - PIN15                     (input pullup).
 */
#define VAL_GPIOB_MODER             (    PIN_MODE_INPUT(GPIOB_0        )        |\
                                         PIN_MODE_INPUT(GPIOB_1        )        |\
                                        PIN_MODE_OUTPUT(GPIOB_2  )     |\
                                     PIN_MODE_ALTERNATE(GPIOB_SPI3_SCK )        |\
                                     PIN_MODE_ALTERNATE(GPIOB_SPI3_MISO)       |\
                                     PIN_MODE_ALTERNATE(GPIOB_SPI3_MOSI)        |\
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SCL )       |\
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SDA )       |\
                                         PIN_MODE_INPUT(GPIOB_ADXL_INT2 )       |\
                                         PIN_MODE_INPUT(GPIOB_ADXL_INT1    )          |\
                                     PIN_MODE_INPUT(GPIOB_SD_CD )       |\
                                     PIN_MODE_ALTERNATE(GPIOB_I2C2_SDA )       |\
                                        PIN_MODE_OUTPUT(GPIOB_FS_CS0  )       |\
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_SCK )       |\
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_MISO)          |\
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_OTYPER            ( PIN_OTYPE_PUSHPULL(GPIOB_0        )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_1        )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_2  )    |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_SPI3_SCK )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_SPI3_MISO)      |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_SPI3_MOSI)       |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SCL )       |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_ADXL_INT2 )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_ADXL_INT1    )       |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_SD_CD )       |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C2_SDA )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_FS_CS0  )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_SPI2_SCK )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_SPI2_MISO)       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_0        )          |\
                                     PIN_OSPEED_HIGH(GPIOB_1        )          |\
                                     PIN_OSPEED_HIGH(GPIOB_2  )       |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI3_SCK )          |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI3_MISO)         |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI3_MOSI)          |\
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SCL )          |\
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SDA )          |\
                                     PIN_OSPEED_HIGH(GPIOB_ADXL_INT2 )          |\
                                     PIN_OSPEED_HIGH(GPIOB_ADXL_INT1    )          |\
                                     PIN_OSPEED_HIGH(GPIOB_SD_CD )          |\
                                     PIN_OSPEED_HIGH(GPIOB_I2C2_SDA )          |\
                                     PIN_OSPEED_HIGH(GPIOB_FS_CS0  )          |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_SCK )          |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_MISO)          |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_0        )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_1        )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_2  )    |\
                                     PIN_PUPDR_FLOATING(GPIOB_SPI3_SCK )       |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_SPI3_MISO)      |\
                                     PIN_PUPDR_FLOATING(GPIOB_SPI3_MOSI)       |\
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SCL )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SDA )       |\
                                     PIN_PUPDR_PULLUP(GPIOB_ADXL_INT2 )       |\
                                     PIN_PUPDR_PULLUP(GPIOB_ADXL_INT1    )       |\
                                     PIN_PUPDR_PULLUP(GPIOB_SD_CD )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_I2C2_SDA )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_FS_CS0  )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_SCK )       |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_SPI2_MISO)       |\
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_0        )             |\
                                     PIN_ODR_HIGH(GPIOB_1        )             |\
                                     PIN_ODR_HIGH(GPIOB_2  )          |\
                                     PIN_ODR_HIGH(GPIOB_SPI3_SCK )             |\
                                     PIN_ODR_HIGH(GPIOB_SPI3_MISO)            |\
                                     PIN_ODR_HIGH(GPIOB_SPI3_MOSI)             |\
                                     PIN_ODR_HIGH(GPIOB_I2C1_SCL )             |\
                                     PIN_ODR_HIGH(GPIOB_I2C1_SDA )             |\
                                     PIN_ODR_HIGH(GPIOB_ADXL_INT2 )             |\
                                     PIN_ODR_HIGH(GPIOB_ADXL_INT1    )             |\
                                     PIN_ODR_HIGH(GPIOB_SD_CD )             |\
                                     PIN_ODR_HIGH(GPIOB_I2C2_SDA )             |\
                                     PIN_ODR_HIGH(GPIOB_FS_CS0  )             |\
                                     PIN_ODR_HIGH(GPIOB_SPI2_SCK )             |\
                                     PIN_ODR_HIGH(GPIOB_SPI2_MISO)             |\
                                     PIN_ODR_HIGH(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_0        ,0)           |\
                                     PIN_AFIO_AF(GPIOB_1        ,0)           |\
                                     PIN_AFIO_AF(GPIOB_2  ,0)        |\
                                     PIN_AFIO_AF(GPIOB_SPI3_SCK ,6)           |\
                                     PIN_AFIO_AF(GPIOB_SPI3_MISO,6)          |\
                                     PIN_AFIO_AF(GPIOB_SPI3_MOSI,6)           |\
                                     PIN_AFIO_AF(GPIOB_I2C1_SCL ,4 )            |\
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA ,4 ))           
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_ADXL_INT2 ,0)           |\
                                     PIN_AFIO_AF(GPIOB_ADXL_INT1    ,0)           |\
                                     PIN_AFIO_AF(GPIOB_SD_CD ,0)           |\
                                     PIN_AFIO_AF(GPIOB_I2C2_SDA ,0)           |\
                                     PIN_AFIO_AF(GPIOB_FS_CS0  ,0 )           |\
                                     PIN_AFIO_AF(GPIOB_SPI2_SCK ,5)           |\
                                     PIN_AFIO_AF(GPIOB_SPI2_MISO,5)            |\
                                     PIN_AFIO_AF(GPIOB_SPI2_MOSI,5))

/*
 * GPIOC setup:
 *
 * PC0  - OTG_FS_POWER_ON           (output pushpull maximum).
 * PC1  - PIN1                      (input pullup).
 * PC2  - PIN2                      (input pullup).
 * PC3  - PDM_OUT                   (input pullup).
 * PC4  - PIN4                      (input pullup).
 * PC5  - PIN5                      (input pullup).
 * PC6  - PIN6                      (input pullup).
 * PC7  - MCLK                      (alternate 6).
 * PC8  - PIN8                      (input pullup).
 * PC9  - PIN9                      (input pullup).
 * PC10 - SCLK                      (alternate 6).
 * PC11 - PIN11                     (input pullup).
 * PC12 - SDIN                      (alternate 6).
 * PC13 - PIN13                     (input pullup).
 * PC14 - PIN14                     (input pullup).
 * PC15 - PIN15                     (input pullup).
 */
#define VAL_GPIOC_MODER             ( PIN_MODE_ANALOG(GPIOC_AD0   )        |\
                                     PIN_MODE_ANALOG(GPIOC_AD1  )        |\
                                     PIN_MODE_ANALOG(GPIOC_AD2  )        |\
                                      PIN_MODE_OUTPUT(GPIOC_LOG_LED)        |\
                                      PIN_MODE_OUTPUT(GPIOC_4)        |\
                                      PIN_MODE_INPUT(GPIOC_5        )        |\
                                      PIN_MODE_OUTPUT(GPIOC_FS_CS1        )        |\
                                      PIN_MODE_OUTPUT(GPIOC_FS_CS2        )        |\
                                  PIN_MODE_ALTERNATE(GPIOC_SDIO_D0  )        |\
                                  PIN_MODE_ALTERNATE(GPIOC_SDIO_D1  )        |\
                                  PIN_MODE_ALTERNATE(GPIOC_SDIO_D2  )        |\
                                  PIN_MODE_ALTERNATE(GPIOC_SDIO_D3  )        |\
                                  PIN_MODE_ALTERNATE(GPIOC_SDIO_CLK )        |\
                                      PIN_MODE_INPUT(GPIOC_USER_BTN  )        |\
                                      PIN_MODE_INPUT(GPIOC_OSC32_IN )        |\
                                      PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_AD0   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_AD1  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_AD2  )        |\
                                     PIN_OTYPE_OPENDRAIN(GPIOC_LOG_LED)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_4)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_5        )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_FS_CS1        )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_FS_CS2        )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D0  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D1  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D2  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D3  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_CLK )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_USER_BTN  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_AD0   )           |\
                                     PIN_OSPEED_HIGH(GPIOC_AD1  )           |\
                                     PIN_OSPEED_HIGH(GPIOC_AD2  )           |\
                                     PIN_OSPEED_HIGH(GPIOC_LOG_LED)           |\
                                     PIN_OSPEED_HIGH(GPIOC_4)           |\
                                     PIN_OSPEED_HIGH(GPIOC_5        )           |\
                                     PIN_OSPEED_HIGH(GPIOC_FS_CS1        )           |\
                                     PIN_OSPEED_HIGH(GPIOC_FS_CS2        )           |\
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D0  )           |\
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D1  )           |\
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D2  )           |\
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D3  )           |\
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_CLK )           |\
                                     PIN_OSPEED_HIGH(GPIOC_USER_BTN  )           |\
                                     PIN_OSPEED_HIGH(GPIOC_OSC32_IN )           |\
                                     PIN_OSPEED_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_AD0   )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_AD1  )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_AD2  )        |\
                                     PIN_PUPDR_FLOATING(GPIOC_LOG_LED )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_4 )        |\
                                       PIN_PUPDR_PULLUP(GPIOC_5        )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_FS_CS1        )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_FS_CS2        )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D0  )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D1  )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D2  )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D3  )        |\
                                     PIN_PUPDR_PULLDOWN(GPIOC_SDIO_CLK )        |\
                                     PIN_PUPDR_PULLUP(GPIOC_USER_BTN  )        |\
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN )        |\
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_AD0   )              |\
                                     PIN_ODR_HIGH(GPIOC_AD1  )              |\
                                     PIN_ODR_HIGH(GPIOC_AD2  )              |\
                                     PIN_ODR_HIGH(GPIOC_LOG_LED)              |\
                                     PIN_ODR_HIGH(GPIOC_4)              |\
                                     PIN_ODR_HIGH(GPIOC_5        )              |\
                                     PIN_ODR_HIGH(GPIOC_FS_CS1        )              |\
                                     PIN_ODR_HIGH(GPIOC_FS_CS2        )              |\
                                     PIN_ODR_HIGH(GPIOC_SDIO_D0  )              |\
                                     PIN_ODR_HIGH(GPIOC_SDIO_D1  )              |\
                                     PIN_ODR_HIGH(GPIOC_SDIO_D2  )              |\
                                     PIN_ODR_HIGH(GPIOC_SDIO_D3  )              |\
                                     PIN_ODR_HIGH(GPIOC_SDIO_CLK )              |\
                                     PIN_ODR_HIGH(GPIOC_USER_BTN  )              |\
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN )              |\
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_AD0   ,0   )               |\
                                     PIN_AFIO_AF(GPIOC_AD1  ,0   )               |\
                                     PIN_AFIO_AF(GPIOC_AD2  ,0  )               |\
                                     PIN_AFIO_AF(GPIOC_LOG_LED,0)               |\
                                     PIN_AFIO_AF(GPIOC_4,0  )               |\
                                     PIN_AFIO_AF(GPIOC_5        ,0  )               |\
                                     PIN_AFIO_AF(GPIOC_FS_CS1        ,0      )               |\
                                     PIN_AFIO_AF(GPIOC_FS_CS2        ,0      ))              
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_SDIO_D0  ,12      )               |\
                                     PIN_AFIO_AF(GPIOC_SDIO_D1  ,12     )               |\
                                     PIN_AFIO_AF(GPIOC_SDIO_D2  ,12  )               |\
                                     PIN_AFIO_AF(GPIOC_SDIO_D3  ,12 )               |\
                                     PIN_AFIO_AF(GPIOC_SDIO_CLK ,12 )               |\
                                     PIN_AFIO_AF(GPIOC_USER_BTN  ,0     )               |\
                                     PIN_AFIO_AF(GPIOC_OSC32_IN ,0  )               |\
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT,0 ))

/*
 * GPIOD setup:
 *
 * PD0  - PIN0                      (input pullup).
 * PD1  - PIN1                      (input pullup).
 * PD2  - PIN2                      (input pullup).
 * PD3  - PIN3                      (input pullup).
 * PD4  - RESET                     (output pushpull maximum).
 * PD5  - OVER_CURRENT              (input floating).
 * PD6  - PIN6                      (input pullup).
 * PD7  - PIN7                      (input pullup).
 * PD8  - PIN8                      (input pullup).
 * PD9  - PIN9                      (input pullup).
 * PD10 - PIN10                     (input pullup).
 * PD11 - PIN11                     (input pullup).
 * PD12 - LED4                      (output pushpull maximum).
 * PD13 - LED3                      (output pushpull maximum).
 * PD14 - LED5                      (output pushpull maximum).
 * PD15 - LED6                      (output pushpull maximum).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_0  )        |\
                                     PIN_MODE_INPUT(GPIOD_1  )        |\
                                     PIN_MODE_ALTERNATE(GPIOD_SDIO_CMD  )        |\
                                     PIN_MODE_INPUT(GPIOD_3  )        |\
                                     PIN_MODE_INPUT(GPIOD_4  )        |\
                                     PIN_MODE_INPUT(GPIOD_5  )        |\
                                     PIN_MODE_INPUT(GPIOD_6  )        |\
                                     PIN_MODE_INPUT(GPIOD_7  )        |\
                                     PIN_MODE_INPUT(GPIOD_8  )        |\
                                     PIN_MODE_INPUT(GPIOD_9  )        |\
                                     PIN_MODE_INPUT(GPIOD_10 )        |\
                                     PIN_MODE_INPUT(GPIOD_11 )        |\
                                     PIN_MODE_INPUT(GPIOD_12 )        |\
                                     PIN_MODE_INPUT(GPIOD_13 )        |\
                                     PIN_MODE_INPUT(GPIOD_14 )        |\
                                     PIN_MODE_INPUT(GPIOD_15 ))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_0 )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_1 )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_SDIO_CMD )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_3 )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_4 )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_5   		)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_6   		)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_7 )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_8   		)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_9   		)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_10)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_11)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_12)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_13)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_14)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_0 	)           |\
                                     PIN_OSPEED_HIGH(GPIOD_1 )           |\
                                     PIN_OSPEED_HIGH(GPIOD_SDIO_CMD  )           |\
                                     PIN_OSPEED_HIGH(GPIOD_3 	)           |\
                                     PIN_OSPEED_HIGH(GPIOD_4 	)           |\
                                     PIN_OSPEED_HIGH(GPIOD_5    		)           |\
                                     PIN_OSPEED_HIGH(GPIOD_6    		)           |\
                                     PIN_OSPEED_HIGH(GPIOD_7 	)           |\
                                     PIN_OSPEED_HIGH(GPIOD_8    		)           |\
                                     PIN_OSPEED_HIGH(GPIOD_9    		)           |\
                                     PIN_OSPEED_HIGH(GPIOD_10)           |\
                                     PIN_OSPEED_HIGH(GPIOD_11)           |\
                                     PIN_OSPEED_HIGH(GPIOD_12)           |\
                                     PIN_OSPEED_HIGH(GPIOD_13)           |\
                                     PIN_OSPEED_HIGH(GPIOD_14)           |\
                                     PIN_OSPEED_HIGH(GPIOD_15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_0 )        |\
                                     PIN_PUPDR_FLOATING(GPIOD_1 )        |\
                                     PIN_PUPDR_PULLUP(GPIOD_SDIO_CMD )        |\
                                     PIN_PUPDR_FLOATING(GPIOD_3 )        |\
                                     PIN_PUPDR_FLOATING(GPIOD_4 )        |\
                                     PIN_PUPDR_FLOATING(GPIOD_5   		)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_6   		)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_7 )        |\
                                     PIN_PUPDR_FLOATING(GPIOD_8   		)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_9   		)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_10)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_11)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_12)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_13)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_14)        |\
                                     PIN_PUPDR_FLOATING(GPIOD_15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_0 )              |\
                                     PIN_ODR_HIGH(GPIOD_1 )              |\
                                     PIN_ODR_HIGH(GPIOD_SDIO_CMD )              |\
                                     PIN_ODR_HIGH(GPIOD_3 )              |\
                                     PIN_ODR_HIGH(GPIOD_4 )              |\
                                     PIN_ODR_HIGH(GPIOD_5 		)              |\
                                     PIN_ODR_HIGH(GPIOD_6 		)              |\
                                     PIN_ODR_HIGH(GPIOD_7 )              |\
                                     PIN_ODR_HIGH(GPIOD_8 		)              |\
                                     PIN_ODR_HIGH(GPIOD_9 		)              |\
                                     PIN_ODR_HIGH(GPIOD_10)            |\
                                     PIN_ODR_HIGH(GPIOD_11)              |\
                                     PIN_ODR_HIGH(GPIOD_12)              |\
                                     PIN_ODR_HIGH(GPIOD_13)              |\
                                     PIN_ODR_HIGH(GPIOD_14)              |\
                                     PIN_ODR_HIGH(GPIOD_15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_0 ,0)           |\
                                     PIN_AFIO_AF(GPIOD_1 ,0)    |\
                                     PIN_AFIO_AF(GPIOD_SDIO_CMD ,12)       |\
                                     PIN_AFIO_AF(GPIOD_3 ,0)     |\
                                     PIN_AFIO_AF(GPIOD_4 ,0)       |\
                                     PIN_AFIO_AF(GPIOD_5 ,0)           |\
                                     PIN_AFIO_AF(GPIOD_6 ,0)               |\
                                     PIN_AFIO_AF(GPIOD_7 ,0))           
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_8 ,0)            |\
                                     PIN_AFIO_AF(GPIOD_9 ,0)           |\
                                     PIN_AFIO_AF(GPIOD_10,0) |\
                                     PIN_AFIO_AF(GPIOD_11,0)|\
                                     PIN_AFIO_AF(GPIOD_12,0)|\
                                     PIN_AFIO_AF(GPIOD_13,0)    |\
                                     PIN_AFIO_AF(GPIOD_14,0) |\
                                     PIN_AFIO_AF(GPIOD_15,0))

/*
 * GPIOE setup:
 *
 * PE0  - INT1                      (input floating).
 * PE1  - INT2                      (input floating).
 * PE2  - PIN2                      (input floating).
 * PE3  - CS_SPI                    (output pushpull maximum).
 * PE4  - PIN4                      (input floating).
 * PE5  - PIN5                      (input floating).
 * PE6  - PIN6                      (input floating).
 * PE7  - PIN7                      (input floating).
 * PE8  - PIN8                      (input floating).
 * PE9  - PIN9                      (input floating).
 * PE10 - PIN10                     (input floating).
 * PE11 - PIN11                     (input floating).
 * PE12 - PIN12                     (input floating).
 * PE13 - PIN13                     (input floating).
 * PE14 - PIN14                     (input floating).
 * PE15 - PIN15                     (input floating).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_0   )        |\
                                     PIN_MODE_INPUT(GPIOE_1   )        |\
                                     PIN_MODE_INPUT(GPIOE_2   )        |\
                                     PIN_MODE_INPUT(GPIOE_3   )        |\
                                     PIN_MODE_INPUT(GPIOE_4   )        |\
                                     PIN_MODE_INPUT(GPIOE_5   )        |\
                                     PIN_MODE_INPUT(GPIOE_6   )        |\
                                     PIN_MODE_INPUT(GPIOE_7   )        |\
                                     PIN_MODE_INPUT(GPIOE_SW11)        |\
                                     PIN_MODE_INPUT(GPIOE_SW12)        |\
                                     PIN_MODE_INPUT(GPIOE_SW13)        |\
                                     PIN_MODE_INPUT(GPIOE_SW14)        |\
                                     PIN_MODE_INPUT(GPIOE_SW21)        |\
                                     PIN_MODE_INPUT(GPIOE_SW22)        |\
                                     PIN_MODE_INPUT(GPIOE_SW23)        |\
                                     PIN_MODE_INPUT(GPIOE_SW24))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_0   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_1   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_2   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_3   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_4   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_5   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_6   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_7   )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SW11)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SW12)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SW13)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SW14)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SW21)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SW22)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SW23)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_SW24))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_0   )           |\
                                     PIN_OSPEED_HIGH(GPIOE_1   )           |\
                                     PIN_OSPEED_HIGH(GPIOE_2   )           |\
                                     PIN_OSPEED_HIGH(GPIOE_3   )           |\
                                     PIN_OSPEED_HIGH(GPIOE_4   )           |\
                                     PIN_OSPEED_HIGH(GPIOE_5   )           |\
                                     PIN_OSPEED_HIGH(GPIOE_6   )           |\
                                     PIN_OSPEED_HIGH(GPIOE_7   )           |\
                                     PIN_OSPEED_HIGH(GPIOE_SW11)           |\
                                     PIN_OSPEED_HIGH(GPIOE_SW12)           |\
                                     PIN_OSPEED_HIGH(GPIOE_SW13)           |\
                                     PIN_OSPEED_HIGH(GPIOE_SW14)           |\
                                     PIN_OSPEED_HIGH(GPIOE_SW21)           |\
                                     PIN_OSPEED_HIGH(GPIOE_SW22)           |\
                                     PIN_OSPEED_HIGH(GPIOE_SW23)           |\
                                     PIN_OSPEED_HIGH(GPIOE_SW24))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_0   )        |\
                                     PIN_PUPDR_FLOATING(GPIOE_1   )        |\
                                     PIN_PUPDR_FLOATING(GPIOE_2   )        |\
                                     PIN_PUPDR_FLOATING(GPIOE_3   )        |\
                                     PIN_PUPDR_FLOATING(GPIOE_4   )        |\
                                     PIN_PUPDR_FLOATING(GPIOE_5   )        |\
                                     PIN_PUPDR_FLOATING(GPIOE_6   )        |\
                                     PIN_PUPDR_FLOATING(GPIOE_7   )        |\
                                     PIN_PUPDR_PULLUP(GPIOE_SW11)        |\
                                     PIN_PUPDR_PULLUP(GPIOE_SW12)        |\
                                     PIN_PUPDR_PULLUP(GPIOE_SW13)        |\
                                     PIN_PUPDR_PULLUP(GPIOE_SW14)        |\
                                     PIN_PUPDR_PULLUP(GPIOE_SW21)        |\
                                     PIN_PUPDR_PULLUP(GPIOE_SW22)        |\
                                     PIN_PUPDR_PULLUP(GPIOE_SW23)        |\
                                     PIN_PUPDR_PULLUP(GPIOE_SW24))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_0   )              |\
                                     PIN_ODR_HIGH(GPIOE_1   )              |\
                                     PIN_ODR_HIGH(GPIOE_2   )              |\
                                     PIN_ODR_HIGH(GPIOE_3   )              |\
                                     PIN_ODR_HIGH(GPIOE_4   )              |\
                                     PIN_ODR_HIGH(GPIOE_5   )              |\
                                     PIN_ODR_HIGH(GPIOE_6   )              |\
                                     PIN_ODR_HIGH(GPIOE_7   )              |\
                                     PIN_ODR_HIGH(GPIOE_SW11)              |\
                                     PIN_ODR_HIGH(GPIOE_SW12)              |\
                                     PIN_ODR_HIGH(GPIOE_SW13)              |\
                                     PIN_ODR_HIGH(GPIOE_SW14)              |\
                                     PIN_ODR_HIGH(GPIOE_SW21)              |\
                                     PIN_ODR_HIGH(GPIOE_SW22)              |\
                                     PIN_ODR_HIGH(GPIOE_SW23)              |\
                                     PIN_ODR_HIGH(GPIOE_SW24))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_0,0   )               |\
                                     PIN_AFIO_AF(GPIOE_1,0 )               |\
                                     PIN_AFIO_AF(GPIOE_2,0)               |\
                                     PIN_AFIO_AF(GPIOE_3,0)              |\
                                     PIN_AFIO_AF(GPIOE_4,0)               |\
                                     PIN_AFIO_AF(GPIOE_5,0)               |\
                                     PIN_AFIO_AF(GPIOE_6,0   )               |\
                                     PIN_AFIO_AF(GPIOE_7,0   ))              
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_SW11,0     )               |\
                                     PIN_AFIO_AF(GPIOE_SW12,0    )               |\
                                     PIN_AFIO_AF(GPIOE_SW13,0 )               |\
                                     PIN_AFIO_AF(GPIOE_SW14,0)               |\
                                     PIN_AFIO_AF(GPIOE_SW21,0)               |\
                                     PIN_AFIO_AF(GPIOE_SW22,0     )               |\
                                     PIN_AFIO_AF(GPIOE_SW23,0  )               |\
                                     PIN_AFIO_AF(GPIOE_SW24,0 ))

/*
 * GPIOF setup:
 *
 * PF0  - PIN0                      (input floating).
 * PF1  - PIN1                      (input floating).
 * PF2  - PIN2                      (input floating).
 * PF3  - PIN3                      (input floating).
 * PF4  - PIN4                      (input floating).
 * PF5  - PIN5                      (input floating).
 * PF6  - PIN6                      (input floating).
 * PF7  - PIN7                      (input floating).
 * PF8  - PIN8                      (input floating).
 * PF9  - PIN9                      (input floating).
 * PF10 - PIN10                     (input floating).
 * PF11 - PIN11                     (input floating).
 * PF12 - PIN12                     (input floating).
 * PF13 - PIN13                     (input floating).
 * PF14 - PIN14                     (input floating).
 * PF15 - PIN15                     (input floating).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0U))

/*
 * GPIOG setup:
 *
 * PG0  - PIN0                      (input floating).
 * PG1  - PIN1                      (input floating).
 * PG2  - PIN2                      (input floating).
 * PG3  - PIN3                      (input floating).
 * PG4  - PIN4                      (input floating).
 * PG5  - PIN5                      (input floating).
 * PG6  - PIN6                      (input floating).
 * PG7  - PIN7                      (input floating).
 * PG8  - PIN8                      (input floating).
 * PG9  - PIN9                      (input floating).
 * PG10 - PIN10                     (input floating).
 * PG11 - PIN11                     (input floating).
 * PG12 - PIN12                     (input floating).
 * PG13 - PIN13                     (input floating).
 * PG14 - PIN14                     (input floating).
 * PG15 - PIN15                     (input floating).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(GPIOG_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_HIGH(GPIOG_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOG_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOG_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOG_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOG_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN15))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN7, 0U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0U))

/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - OSC_OUT                   (input floating).
 * PH2  - PIN2                      (input floating).
 * PH3  - PIN3                      (input floating).
 * PH4  - PIN4                      (input floating).
 * PH5  - PIN5                      (input floating).
 * PH6  - PIN6                      (input floating).
 * PH7  - PIN7                      (input floating).
 * PH8  - PIN8                      (input floating).
 * PH9  - PIN9                      (input floating).
 * PH10 - PIN10                     (input floating).
 * PH11 - PIN11                     (input floating).
 * PH12 - PIN12                     (input floating).
 * PH13 - PIN13                     (input floating).
 * PH14 - PIN14                     (input floating).
 * PH15 - PIN15                     (input floating).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |        \
                                     PIN_MODE_INPUT(GPIOH_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_OSC_IN) |        \
                                     PIN_OSPEED_HIGH(GPIOH_OSC_OUT) |       \
                                     PIN_OSPEED_HIGH(GPIOH_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) |          \
                                     PIN_ODR_HIGH(GPIOH_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0U))

/*
 * GPIOI setup:
 *
 * PI0  - PIN0                      (input floating).
 * PI1  - PIN1                      (input floating).
 * PI2  - PIN2                      (input floating).
 * PI3  - PIN3                      (input floating).
 * PI4  - PIN4                      (input floating).
 * PI5  - PIN5                      (input floating).
 * PI6  - PIN6                      (input floating).
 * PI7  - PIN7                      (input floating).
 * PI8  - PIN8                      (input floating).
 * PI9  - PIN9                      (input floating).
 * PI10 - PIN10                     (input floating).
 * PI11 - PIN11                     (input floating).
 * PI12 - PIN12                     (input floating).
 * PI13 - PIN13                     (input floating).
 * PI14 - PIN14                     (input floating).
 * PI15 - PIN15                     (input floating).
 */
#define VAL_GPIOI_MODER             (PIN_MODE_INPUT(GPIOI_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN15))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN15))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_HIGH(GPIOI_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOI_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOI_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOI_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOI_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_FLOATING(GPIOI_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN15))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN7, 0U))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN15, 0U))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
