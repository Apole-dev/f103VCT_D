<div align="center">

# STM32f103VCT_D 4kW BLDC Motor Driver (Test Code)

![Status: Test Code](https://img.shields.io/badge/Status-Test_Code-yellow?style=flat-square)
![Hardware: Unverified](https://img.shields.io/badge/Hardware-Unverified-red?style=flat-square)
![MCU: STM32F1](https://img.shields.io/badge/MCU-STM32F1xx-blue?style=flat-square)
![Language: C](https://img.shields.io/badge/Language-C-00599C?style=flat-square&logo=c)
![Framework: STM32 HAL](https://img.shields.io/badge/Framework-STM32_HAL-03234B?style=flat-square)

</div>

This repository contains the foundational **open-loop test code** for a motor driver board designed to control a 4kW 3-phase Brushless DC (BLDC) motor. The project is developed using the STM32F1 series microcontroller architecture.

> ⚠️ **CRITICAL WARNING:** **This project is currently in the "Test Code" stage. The software has NOT been 100% verified on the physical custom driver board, and full-load tests have not been completed. Ensure current limits are properly set and adequate safety precautions are taken before running this code on physical hardware.**

## Target Motor Specifications
The driver is designed to operate a motor with the following nominal ratings:
* **Motor Type:** 3-Phase BLDC
* **Nominal Voltage:** 72V
* **Nominal Current:** 55A
* **Power:** 4 kW

## Software Features & Working Principle

The current codebase is structured to test sensorless / open-loop startup and basic hardware protections:

* **Driving Method:** 6-Step (Block) Commutation. Although Hall sensor pins (PD2, PD3, PD4) are initialized, the current driving scheme is strictly **open-loop**.
* **PWM Configuration (TIM1):** 6-channel complementary PWM output switching at 16 kHz.
* **Dead-Time Insertion:** Hardware dead-time of approximately `~2.5 µs` is configured to prevent shoot-through between high-side and low-side MOSFETs.
* **Soft-Start Routine:** Utilizes the `TIM2` timer interrupt (10 kHz / 0.1ms) to align the motor rotor initially, followed by a gradual ramp-up of the PWM duty cycle and commutation frequency.
* **Hardware Fault Protection:** ALM (Alarm) pins (PC10, PC11, PC12) are tied to Falling Edge external interrupts (EXTI). In the event of an overcurrent or an IPM/Driver fault signal, the `__HAL_TIM_MOE_DISABLE` macro is triggered, instantly cutting off all PWM outputs and putting the hardware into a High-Z (safe) state.

## Initialized Peripherals
* **TIM1:** Advanced Control Timer for motor PWM generation.
* **TIM2:** Base Timer for commutation timing and the soft-start ramp algorithm.
* **ADC1:** Channel 2 initialized (reserved for future voltage/current sensing).
* **EXTI:** Hardware interrupts configured for Fault (ALM) signals and Hall sensor inputs.
* **CAN1:** Initialized and standing by for future communication infrastructure.
