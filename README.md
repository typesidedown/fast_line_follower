# Line Follower Robot

A line-following robot project built with PlatformIO for Teensy 4.0 board.

## Description

This project implements a line follower robot using infrared sensors for line detection, an IMU for orientation, an OLED display for status, and motor control for movement.

## Hardware Requirements

- Teensy 4.0 microcontroller
- Infrared line sensors
- BMI160 IMU sensor
- SSD1306 OLED display
- Motor drivers and motors

## Software Requirements

- PlatformIO IDE
- Arduino framework

## Build and Upload

1. Open the project in PlatformIO.
2. Build the project: `pio run`
3. Upload to the board: `pio run --target upload`

## Usage

Power on the robot and place it on a line track. The robot will follow the line autonomously.

## Files

- `src/main.cpp`: Main program loop
- `src/sensors.cpp`: Line sensor handling
- `src/movement.cpp`: Motor control
- `src/imu.cpp`: IMU sensor integration
- `src/oled.cpp`: OLED display management
- `src/config.cpp`: Configuration settings