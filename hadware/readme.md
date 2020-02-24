# Used Hardware

## Body

As a base for the body this project was used <https://www.thingiverse.com/thing:2244952>

## Motion

- 2 MG995 servos as DC-motors for movement, hacked for continuous movement
- 2 Wheels with 40 2mm x 3mm x 1mm magnets
- 2 Hall-Effect Sensor boards as Wheel encoders
- Double Dual H-Bridge for controlling the DC-motors
- 2 INA219 current sensor modules

## Orientation

- 3 Ultrasonic distance sensors
- MPU9250 or MPU6500 for internal orientation measurement
- PMW3506 or similar for surface tacking
- EXPERIMENTAL 360 LIDAR for Testing

## Cleaning

- A2212 Brushless Motor powering the vacuum turbine
- ESC condoling the Brushless Motor
- DC-motor powering the floor brush

## Controller

- STM32F103C8T6(Blue Pill) managing the sonar sensor array
- STM32F446RE NUCLEO-64 as the main platform for the robot
- ESP32 or ESP8266 as wireless logger
- EXPERIMENTAL RaspberryPi analyzing LIDAR data
