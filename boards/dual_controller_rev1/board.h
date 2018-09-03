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
 * Setup for STMicroelectronics STM32 Nucleo144-F429ZI board.
 */

/*
 * Board identifier.
 */
#define BOARD_DUAL_CONTROLLER_REV1
#define BOARD_NAME                  "Dual Controller Rev1"

/*
 * Ethernet PHY type.
 */
#define BOARD_PHY_ID                MII_LAN8742A_ID
#define BOARD_PHY_RMII

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                25000000U
#endif

#define STM32_HSE_BYPASS

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F429xx

/*
 * IO pins assignments.
 */
#define GPIOA_TIM2_CH1              0U
#define GPIOA_RMII_REF_CLK          1U
#define GPIOA_RMII_MDIO             2U
#define GPIOA_ADC123_IN3            3U
#define GPIOA_ADC123_IN4            4U
#define GPIOA_TIM8_BKIN1            6U
#define GPIOA_RMII_RX_DV            7U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_SPI3_CS               15U

#define GPIOB_ADC12_IN8             0U
#define GPIOB_ADC12_IN9             1U
#define GPIOB_TIM2_CH2              3U
#define GPIOB_TIM2_CH3              10U
#define GPIOB_RMII_TXD1             13U

#define GPIOC_RMII_MDC              1U
#define GPIOC_RMII_RXD0             4U
#define GPIOC_RMII_RXD1             5U
#define GPIOC_TIM8_CH1              6U
#define GPIOC_TIM8_CH2              7U
#define GPIOC_TIM8_CH3              8U
#define GPIOC_SPI3_SCK              10U
#define GPIOC_SPI3_MISO             11U
#define GPIOC_SPI3_MOSI             12U

#define GPIOD_TIM4_CH1              12U
#define GPIOD_TIM4_CH2              13U
#define GPIOD_TIM4_CH3              14U

#define GPIOE_TIM1_CH1              9U
#define GPIOE_TIM1_CH2              11U
#define GPIOE_TIM1_CH3              13U
#define GPIOE_TIM1_BKIN1            15U

#define GPIOF_ADC3_4                6U
#define GPIOF_ADC3_5                7U
#define GPIOF_ADC3_6                8U
#define GPIOF_ADC3_7                9U
#define GPIOF_ADC3_8                10U
#define GPIOF_PIN11                 11U
#define GPIOF_PIN12                 12U

#define GPIOG_PIN0                  0U
#define GPIOG_PIN1                  1U
#define GPIOG_RMII_TX_EN            11U
#define GPIOG_RMII_TXD0             13U

/*
 * IO lines assignments.
 */
#define LINE_TIM2_CH1              PAL_LINE(GPIOA, 0U)
#define LINE_RMII_REF_CLK          PAL_LINE(GPIOA, 1U)
#define LINE_RMII_MDIO             PAL_LINE(GPIOA, 2U)
#define LINE_ADC123_IN3            PAL_LINE(GPIOA, 3U)
#define LINE_ADC123_IN4            PAL_LINE(GPIOA, 4U)
#define LINE_TIM8_BKIN1            PAL_LINE(GPIOA, 6U)
#define LINE_RMII_RX_DV            PAL_LINE(GPIOA, 7U)
#define LINE_SWDIO                 PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK                 PAL_LINE(GPIOA, 14U)
#define LINE_SPI3_CS               PAL_LINE(GPIOA, 15U)

#define LINE_ADC12_IN8             PAL_LINE(GPIOB, 0U)
#define LINE_ADC12_IN9             PAL_LINE(GPIOB, 1U)
#define LINE_TIM2_CH2              PAL_LINE(GPIOB, 3U)
#define LINE_TIM2_CH3              PAL_LINE(GPIOB, 10U)
#define LINE_RMII_TXD1             PAL_LINE(GPIOB, 13U)

#define LINE_RMII_MDC              PAL_LINE(GPIOC, 1U)
#define LINE_RMII_RXD0             PAL_LINE(GPIOC, 4U)
#define LINE_RMII_RXD1             PAL_LINE(GPIOC, 5U)
#define LINE_TIM8_CH1              PAL_LINE(GPIOC, 6U)
#define LINE_TIM8_CH2              PAL_LINE(GPIOC, 7U)
#define LINE_TIM8_CH3              PAL_LINE(GPIOC, 8U)
#define LINE_SPI3_SCK              PAL_LINE(GPIOC, 10U)
#define LINE_SPI3_MISO             PAL_LINE(GPIOC, 11U)
#define LINE_SPI3_MOSI             PAL_LINE(GPIOC, 12U)

#define LINE_TIM4_CH1              PAL_LINE(GPIOD, 12U)
#define LINE_TIM4_CH2              PAL_LINE(GPIOD, 13U)
#define LINE_TIM4_CH3              PAL_LINE(GPIOD, 14U)

#define LINE_TIM1_CH1              PAL_LINE(GPIOE, 9U)
#define LINE_TIM1_CH2              PAL_LINE(GPIOE, 11U)
#define LINE_TIM1_CH3              PAL_LINE(GPIOE, 13U)
#define LINE_TIM1_BKIN1            PAL_LINE(GPIOE, 15U)

#define LINE_ADC3_4                PAL_LINE(GPIOF, 6U)
#define LINE_ADC3_5                PAL_LINE(GPIOF, 7U)
#define LINE_ADC3_6                PAL_LINE(GPIOF, 8U)
#define LINE_ADC3_7                PAL_LINE(GPIOF, 9U)
#define LINE_ADC3_8                PAL_LINE(GPIOF, 10U)
#define LINE_GPIOF_PIN11           PAL_LINE(GPIOF, 11U)
#define LINE_GPIOF_PIN12           PAL_LINE(GPIOF, 12U)

#define LINE_GPIOG_PIN0            PAL_LINE(GPIOG, 0U)
#define LINE_GPIOG_PIN1            PAL_LINE(GPIOG, 1U)
#define LINE_RMII_TX_EN            PAL_LINE(GPIOG, 11U)
#define LINE_RMII_TXD0             PAL_LINE(GPIOG, 13U)


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
 * GPIOA
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ALTERNATE(GPIOA_TIM2_CH1) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_RMII_REF_CLK) |\
                                     PIN_MODE_ALTERNATE(GPIOA_RMII_MDIO) |  \
                                     PIN_MODE_ANALOG(GPIOA_ADC123_IN3) |    \
                                     PIN_MODE_ANALOG(GPIOA_ADC123_IN4) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_TIM8_BKIN1) | \
                                     PIN_MODE_ALTERNATE(GPIOA_RMII_RX_DV) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_OUTPUT(GPIOA_SPI3_CS))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_RMII_MDIO) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI3_CS))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_RMII_MDIO) |     \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SPI3_CS))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIOA_TIM2_CH1) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_RMII_REF_CLK) |\
                                     PIN_PUPDR_PULLUP(GPIOA_RMII_MDIO) |    \
                                     PIN_PUPDR_PULLDOWN(GPIOA_TIM8_BKIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOA_RMII_RX_DV) | \
                                     PIN_PUPDR_FLOATING(GPIOA_SWDIO) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_PULLUP(GPIOA_SPI3_CS))
#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_SPI3_CS))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_TIM2_CH1, 1U)      | \
                                     PIN_AFIO_AF(GPIOA_RMII_REF_CLK, 11U) | \
                                     PIN_AFIO_AF(GPIOA_RMII_MDIO, 11U) |    \
                                     PIN_AFIO_AF(GPIOA_TIM8_BKIN1, 3U) |    \
                                     PIN_AFIO_AF(GPIOA_RMII_RX_DV, 11U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U))

/*
 * GPIOB
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ANALOG(GPIOB_ADC12_IN8) |     \
                                     PIN_MODE_ANALOG(GPIOB_ADC12_IN9) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_TIM2_CH2) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_TIM2_CH3) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_RMII_TXD1))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_RMII_TXD1))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_RMII_TXD1))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLUP(GPIOB_TIM2_CH2) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_TIM2_CH3))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_RMII_TXD1))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_TIM2_CH2, 1U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_TIM2_CH3, 1U)  |     \
                                     PIN_AFIO_AF(GPIOB_RMII_TXD1, 11U))


/*
 * GPIOC
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ALTERNATE(GPIOC_RMII_MDC) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_RMII_RXD0) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_RMII_RXD1) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_TIM8_CH1) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_TIM8_CH2) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_TIM8_CH3) |   \
                                     PIN_MODE_OUTPUT(GPIOC_SPI3_SCK) |      \
                                     PIN_MODE_INPUT(GPIOC_SPI3_MISO) |      \
                                     PIN_MODE_OUTPUT(GPIOC_SPI3_MOSI))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_RMII_MDC) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_TIM8_CH1) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_TIM8_CH2) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_TIM8_CH3) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SPI3_SCK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SPI3_MOSI))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_RMII_MDC) |      \
                                     PIN_OSPEED_HIGH(GPIOC_TIM8_CH1) |      \
                                     PIN_OSPEED_HIGH(GPIOC_TIM8_CH2) |      \
                                     PIN_OSPEED_HIGH(GPIOC_TIM8_CH3) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SPI3_SCK) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SPI3_MOSI))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_RMII_MDC) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_RMII_RXD0) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_RMII_RXD1) |  \
                                     PIN_PUPDR_PULLUP(GPIOC_SPI3_MISO))
#define VAL_GPIOC_ODR               (PIN_ODR_LOW(GPIOC_TIM8_CH1) |          \
                                     PIN_ODR_HIGH(GPIOC_TIM8_CH2) |         \
                                     PIN_ODR_HIGH(GPIOC_TIM8_CH3))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_RMII_MDC, 11U) |     \
                                     PIN_AFIO_AF(GPIOC_RMII_RXD0, 11U) |    \
                                     PIN_AFIO_AF(GPIOC_RMII_RXD1, 11U) |    \
                                     PIN_AFIO_AF(GPIOC_TIM8_CH1, 3U) |    \
                                     PIN_AFIO_AF(GPIOC_TIM8_CH2, 3U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_TIM8_CH3, 3U))


/*
 * GPIOD
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIOD_TIM4_CH1) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_TIM4_CH2) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_TIM4_CH3))
#define VAL_GPIOD_OTYPER            (0)
#define VAL_GPIOD_OSPEEDR           (0)
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLUP(GPIOD_TIM4_CH1) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_TIM4_CH2) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_TIM4_CH3))
#define VAL_GPIOD_ODR               (0)
#define VAL_GPIOD_AFRL              (0)
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_TIM4_CH1, 2U) |       \
                                     PIN_AFIO_AF(GPIOD_TIM4_CH2, 2U) |       \
                                     PIN_AFIO_AF(GPIOD_TIM4_CH3, 2U))


/*
 * GPIOE
 */
#define VAL_GPIOE_MODER             (PIN_MODE_ALTERNATE(GPIOE_TIM1_CH1) |    \
                                     PIN_MODE_ALTERNATE(GPIOE_TIM1_CH2) |    \
                                     PIN_MODE_ALTERNATE(GPIOE_TIM1_CH3) |    \
                                     PIN_MODE_ALTERNATE(GPIOE_TIM1_BKIN1))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_TIM1_CH1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_TIM1_CH2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_TIM1_CH3))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_TIM1_CH1) |       \
                                     PIN_OSPEED_HIGH(GPIOE_TIM1_CH2) |       \
                                     PIN_OSPEED_HIGH(GPIOE_TIM1_CH3))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOE_TIM1_BKIN1))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_TIM1_CH1) |          \
                                     PIN_ODR_HIGH(GPIOE_TIM1_CH2) |          \
                                     PIN_ODR_HIGH(GPIOE_TIM1_CH3))
#define VAL_GPIOE_AFRL              (0)
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_TIM1_CH1, 1U) |       \
                                     PIN_AFIO_AF(GPIOE_TIM1_CH2, 1U) |       \
                                     PIN_AFIO_AF(GPIOE_TIM1_CH3, 1U) |       \
                                     PIN_AFIO_AF(GPIOE_TIM1_BKIN1, 1U))


/*
 * GPIOF
 */
#define VAL_GPIOF_MODER             (PIN_MODE_ANALOG(GPIOF_ADC3_4) |        \
                                     PIN_MODE_ANALOG(GPIOF_ADC3_5) |        \
                                     PIN_MODE_ANALOG(GPIOF_ADC3_6) |        \
                                     PIN_MODE_ANALOG(GPIOF_ADC3_7) |         \
                                     PIN_MODE_ANALOG(GPIOF_ADC3_8) |         \
                                     PIN_MODE_INPUT(GPIOF_PIN11) |         \
                                     PIN_MODE_OUTPUT(GPIOF_PIN12))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_PIN12))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_PIN12))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOF_PIN11))
#define VAL_GPIOF_ODR               (PIN_ODR_LOW(GPIOF_PIN12))
#define VAL_GPIOF_AFRL              (0)
#define VAL_GPIOF_AFRH              (0)



#define GPIOG_PIN0                  0U
#define GPIOG_PIN1                  1U
#define GPIOG_RMII_TX_EN            11U
#define GPIOG_RMII_TXD0             13U

/*
 * GPIOG
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(GPIOG_PIN0) |           \
                                     PIN_MODE_OUTPUT(GPIOG_PIN1) |          \
                                     PIN_MODE_ALTERNATE(GPIOG_RMII_TX_EN) | \
                                     PIN_MODE_ALTERNATE(GPIOG_RMII_TXD0))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_RMII_TX_EN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_RMII_TXD0))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_HIGH(GPIOG_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOG_RMII_TX_EN) |    \
                                     PIN_OSPEED_HIGH(GPIOG_RMII_TXD0))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOG_PIN0))
#define VAL_GPIOG_ODR               (PIN_ODR_LOW(GPIOG_PIN1) |              \
                                     PIN_ODR_HIGH(GPIOG_RMII_TX_EN) |       \
                                     PIN_ODR_HIGH(GPIOG_RMII_TXD0))
#define VAL_GPIOG_AFRL              (0)
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_RMII_TX_EN, 11U) |   \
                                     PIN_AFIO_AF(GPIOG_RMII_TXD0, 11U))

#define VAL_GPIOH_MODER             0
#define VAL_GPIOH_OTYPER            0
#define VAL_GPIOH_OSPEEDR           0
#define VAL_GPIOH_PUPDR             0
#define VAL_GPIOH_ODR               0
#define VAL_GPIOH_AFRL              0
#define VAL_GPIOH_AFRH              0

#define VAL_GPIOI_MODER             0
#define VAL_GPIOI_OTYPER            0
#define VAL_GPIOI_OSPEEDR           0
#define VAL_GPIOI_PUPDR             0
#define VAL_GPIOI_ODR               0
#define VAL_GPIOI_AFRL              0
#define VAL_GPIOI_AFRH              0

#define VAL_GPIOJ_MODER             0
#define VAL_GPIOJ_OTYPER            0
#define VAL_GPIOJ_OSPEEDR           0
#define VAL_GPIOJ_PUPDR             0
#define VAL_GPIOJ_ODR               0
#define VAL_GPIOJ_AFRL              0
#define VAL_GPIOJ_AFRH              0

#define VAL_GPIOK_MODER             0
#define VAL_GPIOK_OTYPER            0
#define VAL_GPIOK_OSPEEDR           0
#define VAL_GPIOK_PUPDR             0
#define VAL_GPIOK_ODR               0
#define VAL_GPIOK_AFRL              0
#define VAL_GPIOK_AFRH              0

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
