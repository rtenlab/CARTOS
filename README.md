# CARTOS: Charging-Aware Real-Time Operating System for Intermittent Batteryless Devices

This repository contains the source code and implementation of **CARTOS**, a charging-aware real-time operating system designed for intermittently-powered batteryless devices (IPDs) in IoT applications. CARTOS ensures reliable real-time scheduling and energy-efficient operation of devices that harvest energy from ambient sources, overcoming the challenges posed by intermittent power supply.

## Features

- **Mixed-preemption Scheduling Model**: Tasks are classified into preemptible (computation) and non-preemptible (peripheral) tasks, allowing the system to handle intermittent power efficiently.
- **Just-In-Time (JIT) Checkpointing**: Computational tasks are dynamically checkpointed when the system detects low energy levels, ensuring forward progress without unnecessary overhead.
- **Peripheral Task Management**: Non-preemptible peripheral tasks are executed atomically, ensuring that critical operations like sensor reading or data logging are not interrupted by power loss.
- **Portability**: CARTOS is built on FreeRTOS and can be easily ported to other embedded real-time operating systems.
- **Energy Prediction and Adaptation**: CARTOS adapts task scheduling to environmental energy availability using a neural network-based energy predictor, ensuring continuous operation in fluctuating conditions.

## System Architecture

The system consists of the following key components:

- **Energy-Harvesting Unit**: Collects energy from ambient sources such as solar, RF, or thermoelectric power.
- **Power Management Unit**: Regulates and manages energy storage, ensuring efficient use of harvested energy.
- **Microcontroller Unit (MCU)**: Executes tasks based on the mixed-preemption model and handles checkpointing, scheduling, and peripheral task management.
- **Memory**: Includes volatile (RAM) and non-volatile (FRAM/MRAM) memory for state saving and checkpointing during power loss.

## Installation

1. Clone this repository:

    ```bash
    git clone https://github.com/rtenlab/CARTOS.git
    ```

2. Ensure you have the necessary development environment set up, including:

    - A compatible MCU development board (e.g., ARM Cortex-M4)
    - FreeRTOS or any other supported RTOS
    - Required peripherals (sensors, capacitors for energy storage, etc.)

3. Build and flash the code to your device using your preferred IDE (e.g., Keil, GCC, IAR).

## Usage

1. **Configuring Tasks**: CARTOS provides APIs for defining tasks as preemptible or non-preemptible. Tasks can be scheduled periodically and grouped into processing chains. Refer to the `task_config.h` file for task configuration options.

    Example:

    ```c
    void Task1(void const * arguments) {
        setTaskPeriod(10000, 2000);  // Period = 10s, Offset = 2s
        while (1) {
            // Task code here
            waitForNextPeriod();
        }
    }
    ```

2. **Running the System**: Once the tasks are defined, compile and run the system on your device. CARTOS will handle energy monitoring, checkpointing, and scheduling based on the available energy.

3. **Energy Prediction**: CARTOS uses an internal energy prediction model to adapt the scheduling strategy based on real-time energy availability.
