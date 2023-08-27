# BMS BQ76920 Library for STM32 MCU

This library provides a comprehensive Battery Monitoring AFE (Analog Front End) solution for STM32 microcontrollers, specifically designed to work with the BQ76920 IC from Texas Instruments. The library leverages the STM32 HAL (Hardware Abstraction Library) platform to simplify integration and enable seamless communication between the STM32 MCU and the BQ76920.

## Features

- **Compatibility**: Designed for use with 4S (4-series) Lithium Batteries.
- **Voltage Monitoring**: Measure individual cell voltages (Vcells) and pack voltage (Vpack).
- **Current Monitoring**: Read pack current for accurate monitoring.
- **Protection**: Overvoltage (OV) and undervoltage (UV) protection mechanisms for enhanced safety.
- **Cell Balancing**: Support for cell balancing to equalize cell voltages.
- **State of Charge (SOC)**: Obtain State of Charge information to estimate battery charge level.
- **State of Health (SOH)**: Calculate State of Health to assess battery health using Kalman filter algorithms.

## Getting Started

1. **Hardware Setup**: Connect the STM32 MCU to the BQ76920 IC according to the hardware specifications provided in your datasheets.

2. **Library Integration**: Add the provided library to your STM32 project by including the necessary files and configuring the build settings.

3. **Initialization**: Initialize the BQ76920 library and STM32 HAL drivers in your code. Configure the necessary pins, I2C communication interfaces, and other settings.

4. **Usage**: Utilize the library functions to interact with the BQ76920 IC. Access cell voltages, pack voltage, current, SOC, SOH, and manage cell balancing as needed.

## Example Code

```c
#include "main.h"
#include "BQ76920.h"
#include <stdio.h>

// Struct
BQ76920_t BMS;
// Important
float PackCurrent = 0, Vcell[4], Vpack, SOC, SOH;
// Alert
uint8_t CC_READY = 0, DEVICE_XREADY = 0, OVRD_ALERT = 0, UV = 0, OV = 0,
		SCD = 0, OCD = 0, CHECK_SOH = 1, UPDATE_SOH = 1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);


int main(void) {
    HAL_Init(); // Reset of all peripherals, Initializes the Flash interface and the Systick
    SystemClock_Config(); // Configure the system clock

    MX_GPIO_Init(); // Initialize all configured peripherals
    MX_I2C2_Init();
    MX_USART2_UART_Init();

    BMS.i2cHandle = &hi2c2;
    BQ76920_Initialise(&BMS, &hi2c2); // Init BMS

    while (1) {
        for (int i = 0; i <= 6; i += 2) { // Get V cells
            Vcell[i / 2] = getCellVoltage(&BMS, 12 + i);
        }

        Vpack = getPackVoltage(&BMS); // Get V pack

        PackCurrent = getCurrent(&BMS); // in mA

        SOC = SOCPack(&BMS, PackCurrent, Vpack);

        SOH = SOHPack(&BMS);

    }
}
```

## Contact
For any inquiries, feedback, or support related to this library, please feel free to contact me at ali.alamsyahdj@gmail.com.
