/**
 * @file hexnet_io_map.h
 * @brief Hexnet IO Module v0.60 – ESP32-S3-WROOM-1U-N8R8 pin map
 *
 * Donanim: 12 role (shift register) + 8 dimmer (MOSFET PWM)
 * Kaynak: guncel sema (IC1 + CAN interface).
 *
 *  ┌────────┬──────────────────────────────────────────────────────────────┐
 *  │  GPIO  │  Function                                                    │
 *  ├────────┼──────────────────────────────────────────────────────────────┤
 *  │   1    │  SensorDetect (sema)                                         │
 *  │   2    │  WS2812 RGB data                                             │
 *  │   3    │  ADC – AnalogInputMCU1                                       │
 *  │   4    │  ADC – Input_Voltage_MCU                                     │
 *  │   5    │  Dimmer PWM #1 (Mosfet-MCU-1)                                │
 *  │   6    │  Dimmer PWM #2 (Mosfet-MCU-2)                                │
 *  │   7    │  Dimmer PWM #3 (Mosfet-MCU-3)                                │
 *  │   8    │  Dimmer PWM #4 (Mosfet-MCU-4)                                │
 *  │   9    │  ADC – AnalogInputMCU3                                       │
 *  │  10    │  (free / Analog3 yedek)                                      │
 *  │  11    │  Motor1 IN A                                                 │
 *  │  12    │  Motor1 IN B                                                 │
 *  │  13    │  Motor2 IN A                                                 │
 *  │  14    │  Motor2 IN B                                                 │
 *  │  15    │  Dimmer PWM #5 (Mosfet-MCU-5)                                │
 *  │  16    │  Dimmer PWM #6 (Mosfet-MCU-6)                                │
 *  │  17    │  Dimmer PWM #7 (Mosfet-MCU-7)                                │
 *  │  18    │  Dimmer PWM #8 (Mosfet-MCU-8)                                │
 *  │  19    │  Status LED (green)                                          │
 *  │  20    │  (NC / reserved)                                             │
 *  │  21    │  Motor1+Motor2 PWM / enable                                  │
 *  │  35    │  ShiftReg STB                                                │
 *  │  36    │  ShiftReg DATA                                               │
 *  │  37    │  ShiftReg CLK                                                │
 *  │  38    │  UART1 TX – RS485                                            │
 *  │  39    │  UART1 RX – RS485                                            │
 *  │  40    │  RS485 DE/RE                                                 │
 *  │  41    │  ESP32_TX_MCU (sema)                                         │
 *  │  42    │  ESP32_RX_MCU (sema)                                         │
 *  │  43    │  U0TXD – ROM flash / console                                 │
 *  │  44    │  U0RXD – ROM flash / console                                 │
 *  │  45    │  (yedek / eski SensorDetect map)                             │
 *  │  46    │  Display 5V enable (default ON)                              │
 *  │  47    │  CAN RX (PCB; sema etiketi TX idi)                           │
 *  │  48    │  CAN TX (PCB; sema etiketi RX idi)                           │
 *  └────────┴──────────────────────────────────────────────────────────────┘
 *
 * Role maskesi: bit0..bit11 → 12 role (shift register zinciri, 16-bit shift).
 * Dimmer: 8x LEDC → Mosfet-MCU-1..8.
 */
#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"

/* --- Kapasite --- */
#define HEXNET_IO_RELAY_COUNT_DEFAULT   12
#define HEXNET_IO_DIM_COUNT_DEFAULT     8

/* --- Status LED --- */
#define HEXNET_IO_RUN_LED_GPIO          GPIO_NUM_19

/* --- Shift register (12 role / 16-bit zincir) — sema IO37/36/35 --- */
#define HEXNET_IO_SR_CLK_GPIO           GPIO_NUM_37
#define HEXNET_IO_SR_DATA_GPIO          GPIO_NUM_36
#define HEXNET_IO_SR_STB_GPIO           GPIO_NUM_35

/* --- Motor (sema: M1 A/B=11/12, M2 A/B=13/14, ortak PWM=21) --- */
#define HEXNET_IO_MOTOR_EN_GPIO         GPIO_NUM_21  /* M1+M2 ortak PWM */
#define HEXNET_IO_MOTOR_A_GPIO          GPIO_NUM_11  /* M1 IN A */
#define HEXNET_IO_MOTOR_B_GPIO          GPIO_NUM_12  /* M1 IN B */
#define HEXNET_IO_MOTOR2_A_GPIO         GPIO_NUM_13  /* M2 IN A */
#define HEXNET_IO_MOTOR2_B_GPIO         GPIO_NUM_14  /* M2 IN B */

/* --- Dimmer PWM (LEDC) — Mosfet-MCU-1..8 = IO5..8, IO15..18 --- */
#define HEXNET_IO_DIM_PWM_GPIO_1        GPIO_NUM_5
#define HEXNET_IO_DIM_PWM_GPIO_2        GPIO_NUM_6
#define HEXNET_IO_DIM_PWM_GPIO_3        GPIO_NUM_7
#define HEXNET_IO_DIM_PWM_GPIO_4        GPIO_NUM_8
#define HEXNET_IO_DIM_PWM_GPIO_5        GPIO_NUM_15
#define HEXNET_IO_DIM_PWM_GPIO_6        GPIO_NUM_16
#define HEXNET_IO_DIM_PWM_GPIO_7        GPIO_NUM_17
#define HEXNET_IO_DIM_PWM_GPIO_8        GPIO_NUM_18

/* --- WS2812 RGB --- */
#define HEXNET_IO_RGB_STRIP_GPIO        GPIO_NUM_2
#define HEXNET_IO_RGB_LED_COUNT         65
#define HEXNET_IO_RGB_MATRIX_ROWS       10
#define HEXNET_IO_RGB_MATRIX_COLS       20
#define HEXNET_IO_RGB_MATRIX_LEDS       (HEXNET_IO_RGB_MATRIX_ROWS * HEXNET_IO_RGB_MATRIX_COLS)

/* --- Sensor detect (sema IO1) --- */
#define HEXNET_IO_SENSOR_DETECT_GPIO    GPIO_NUM_1

/* --- Display 5V rail enable (IO → Display besleme). Aktif-high, boot'ta ON. --- */
#define HEXNET_IO_DISP_5V_EN_GPIO       GPIO_NUM_46
#define HEXNET_IO_DISP_5V_EN_ACTIVE     1
#define HEXNET_IO_DISP_HARD_RESET_MS    3000

/* --- ADC (yazilim kanal eslemesi adc_monitor icinde) --- */
#define HEXNET_IO_ADC_ANALOG1_GPIO      GPIO_NUM_3
/* Analog2 olcum: GPIO9 (sema etiketi IO46 artik Display 5V EN). */
#define HEXNET_IO_ADC_ANALOG2_GPIO      GPIO_NUM_9
#define HEXNET_IO_ADC_ANALOG3_GPIO      GPIO_NUM_10
#define HEXNET_IO_ADC_VIN_GPIO          GPIO_NUM_4

/* --- Eski MUX yok (motor pinleri ile cakisirdi); board init no-op mask --- */
#define HEXNET_IO_MUX_S0_GPIO           GPIO_NUM_NC
#define HEXNET_IO_MUX_S1_GPIO           GPIO_NUM_NC
#define HEXNET_IO_MUX_S2_GPIO           GPIO_NUM_NC
#define HEXNET_IO_MUX_Z_A_GPIO          GPIO_NUM_NC
#define HEXNET_IO_MUX_Z_B_GPIO          GPIO_NUM_NC
#define HEXNET_IO_MUX_CHANNEL_COUNT     0

/* --- DHT kullanilmiyor (iklim SoT = RS485). SensorDetect pin rezerv. --- */
#define HEXNET_IO_DHT_GPIO_PRIMARY      GPIO_NUM_NC
#define HEXNET_IO_DHT_GPIO_ALT          GPIO_NUM_NC

/* --- RS485 / UART1 --- */
#define HEXNET_IO_RS485_EN_GPIO         GPIO_NUM_40
#define HEXNET_IO_UART_RS485_PORT       UART_NUM_1
#define HEXNET_IO_UART_RS485_TX_GPIO    GPIO_NUM_38
#define HEXNET_IO_UART_RS485_RX_GPIO    GPIO_NUM_39
#define HEXNET_IO_UART_RS485_BAUD       115200
#define HEXNET_IO_UART_RS485_BUF_SIZE   512

/* --- CAN / TWAI — PCB'de sema etiketinin tersi (swap ile Bus-Off kalkti) --- */
#define HEXNET_IO_CAN_TX_GPIO           GPIO_NUM_48
#define HEXNET_IO_CAN_RX_GPIO           GPIO_NUM_47

/* --- Debug UART: konsol U0 (43/44). Semada 41/42 = ESP32_TX/RX_MCU. --- */
#define HEXNET_IO_DEBUG_UART_TX_GPIO    GPIO_NUM_NC
#define HEXNET_IO_DEBUG_UART_RX_GPIO    GPIO_NUM_NC

/* Bit masks for gpio_config */
#define HEXNET_IO_MUX_SEL_MASK          (0ULL)
#define HEXNET_IO_MUX_Z_MASK            (0ULL)
#define HEXNET_IO_SR_MASK \
    ((1ULL << HEXNET_IO_SR_CLK_GPIO) | (1ULL << HEXNET_IO_SR_DATA_GPIO) | (1ULL << HEXNET_IO_SR_STB_GPIO))
#define HEXNET_IO_MOTOR_MASK \
    ((1ULL << HEXNET_IO_MOTOR_EN_GPIO) | (1ULL << HEXNET_IO_MOTOR_A_GPIO) | (1ULL << HEXNET_IO_MOTOR_B_GPIO) | \
     (1ULL << HEXNET_IO_MOTOR2_A_GPIO) | (1ULL << HEXNET_IO_MOTOR2_B_GPIO))
#define HEXNET_IO_DIM_GPIO_BIT(g) \
    (((g) >= 0 && (g) != GPIO_NUM_NC) ? (1ULL << (unsigned)(g)) : 0ULL)
#define HEXNET_IO_DIM_PWM_MASK \
    (HEXNET_IO_DIM_GPIO_BIT(HEXNET_IO_DIM_PWM_GPIO_1) | HEXNET_IO_DIM_GPIO_BIT(HEXNET_IO_DIM_PWM_GPIO_2) | \
     HEXNET_IO_DIM_GPIO_BIT(HEXNET_IO_DIM_PWM_GPIO_3) | HEXNET_IO_DIM_GPIO_BIT(HEXNET_IO_DIM_PWM_GPIO_4) | \
     HEXNET_IO_DIM_GPIO_BIT(HEXNET_IO_DIM_PWM_GPIO_5) | HEXNET_IO_DIM_GPIO_BIT(HEXNET_IO_DIM_PWM_GPIO_6) | \
     HEXNET_IO_DIM_GPIO_BIT(HEXNET_IO_DIM_PWM_GPIO_7) | HEXNET_IO_DIM_GPIO_BIT(HEXNET_IO_DIM_PWM_GPIO_8))
