# STM32-Flight-Controller-Core-Bare-Metal-FreeRTOS-
What this project does: This firmware turns an STM32F4 microcontroller into the core of a flight controller. It continuously polls motion data from an MPU6050 (accelerometer/gyroscope) and altitude data from a BME280 sensor. Using a custom Time-Triggered FreeRTOS architecture running at 250Hz, it fuses these raw signals with a Complementary Filter to accurately estimate the aircraft's attitude (Pitch & Roll) in real-time. The calculated telemetry is then visualized on an OLED display, with all low-level communication handled by custom-written, bare-metal I2C drivers for maximum efficiency.

The system runs a deterministic 250Hz control loop, ensuring precise timing for flight stability.

# Key Features
Bare-Metal Drivers: Custom-written I2C drivers using direct register access (CMSIS) for maximum efficiency and control. No HAL overhead.

Time-Triggered Architecture: Utilizes FreeRTOS vTaskDelayUntil to create a jitter-free 250Hz loop, critical for PID stability.

Thread Safety: Resource sharing (I2C Bus) is protected via Mutexes to prevent race conditions between the Display and Sensor tasks.

Sensor Fusion: Implements a Complementary Filter to fuse noisy Accelerometer data with drifting Gyroscope data for stable Pitch/Roll estimation.

Signal Processing: Includes Exponential Moving Average (EMA) filters for the Barometer and Low-Pass Filters (LPF) for the Accelerometer.

# Hardware Requirements
MCU: STM32F411 (BlackPill / Discovery) or compatible STM32F4 series.

IMU: MPU6050 (Accelerometer + Gyroscope).

Barometer: BME280 (Altitude/Pressure).

Display: SSD1306 OLED (128x64 I2C).

# Connection Scheme (Pinout)

<img width="513" height="486" alt="image" src="https://github.com/user-attachments/assets/e7ea48de-9434-41cd-9f08-20b91cd86517" />


# Software Architecture

```text
    [ SENSOR TASK ]  <-- (Priority: High / 250 Hz)
           |
           | (Writes Raw Data)
           v
    ( Raw Data Queue )
           |
           | (Reads Raw Data)
           v
    [ FUSION TASK ]  <-- (Priority: Medium / Event-Driven)
           |
           | (Calculates Pitch/Roll)
           v
    ( Display Queue )
           |
           | (Reads Angles)
           v
    [ DISPLAY TASK ] <-- (Priority: Low / ~25 Hz)
           |
           v
      [ OLED SCREEN ]

```

### System Data Flow



1. Sensor Task (Priority: High)
    Frequency: 250 Hz (4ms period).

    Responsibility: Reads MPU6050 (every cycle) and BME280 (every 10th cycle). Applies initial filtering and sends raw data to the queue.

    Synchronization: Uses Mutex to lock the I2C bus.

2. Fusion Task (Priority: Medium)
    Trigger: Data reception from Queue.

    Responsibility: Calculates dt (delta time), applies Low Pass Filters, and executes the Complementary Filter algorithm to determine attitude (Pitch/Roll).

3. Display Task (Priority: Low)
    Frequency: Limited to ~25Hz.

    Responsibility: Formats the calculated data into strings and updates the SSD1306 OLED screen.


# NOTE
This project was built by integrating custom bare-metal logic with standard industry resources.

SSD1306 Driver: The OLED display driver logic was adapted from open-source community examples and modified to work with the custom bare-metal I2C driver used in this project.

Sensor Fusion Logic: The mathematical formulas for the Complementary Filter and Attitude Estimation (Euler Angles) are based on standard aerospace engineering literature and open-source control theory documentation.

Datasheets: All device register addresses (Hex values), command bytes, and configuration settings were derived directly from the official manufacturer datasheets:

InvenSense MPU-6000/MPU-6050 Product Specification

Bosch Sensortec BME280 Data Sheet

Solomon Systech SSD1306 Datasheet








