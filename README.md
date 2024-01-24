## Command & Control firmware for Quad-copter

The Control program (`runner`) is run on the local machine. It is responsible for: loading the drone code on micro-controller, communicating with the drone, displaying telemetry data and controlling the drone with the joystick. 

The Firmware (`dronecode`) is loaded on the micro-controller (compilation target `thumbv6m-none-eabi`) of the quad-copter. Its purpose is to: read sensor data, control the motors, communicate with the `runner`, send telemetry data, receive commands and run the stabilization algorithm.

