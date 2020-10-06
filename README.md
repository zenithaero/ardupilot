# Setup

## Hardware installation

![alt text](Zenith/Docs/hardware.jpg?raw=true "Hardware wiring")

[Link to the documentation](https://docs.px4.io/master/en/flight_controller/pixhawk4.html)

The pixhawk 4 system is comprised of
- **A**: The pixhawk unit
- **B**: The power module

### Pixhawk periferals

[Link to the documentation](https://docs.px4.io/master/en/peripherals/)

- The receiver (**3**) is connected to the **DSM/SBUS RC** port
- The pressure sensor is connected to the **I2C A** port
- The GPS is connected to the **GPS MODULE** port
- The telemetry module is connected to the **TELEM 1** port

### Power module

[Link to the documentation](https://docs.px4.io/master/en/power_module/holybro_pm07_pixhawk4_power_module.html)

![alt text](https://docs.px4.io/master/assets/hardware/power_module/holybro_pm07/pixhawk4_power_management_board.png "Power board wiring")

The power module provides power to the pixhawk and the servos. It's connected to the pixhawk with a 6-pin power cable as well as a 10-pin PWM cable from the pixawk output **I/O PWM OUT** to the power module input **FMU PWM IN**\
Power comes to the power module from a 7 to 51v (2 to 12s LiPo) DC port (**1**)\
ESCs & servos are connected to the power module at the **FMU PWM OUT** 3-pin rail (**2**). At least one ESC must provide 5V UBEC power to the power rail\
