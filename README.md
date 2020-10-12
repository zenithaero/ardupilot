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

The power module provides power to the pixhawk and the servos. It's connected to the pixhawk with a 6-pin power cable as well as a 10-pin PWM cable from the pixawk output **I/O-PWM-OUT** to the power module input **FMU-PWM-IN**\
Power comes to the power module from a 7 to 51v (2 to 12s LiPo) DC port (**1**)\
ESCs & servos are connected to the power module at the **FMU-PWM-OUT** 3-pin rail (**2**). At least one ESC must provide 5V UBEC power to the power rail


## Software installation

# Cloning the code and setting up the environment

Make sure [brew](https://brew.sh/) and [python](https://docs.brew.sh/Homebrew-and-Python) are installed. If not, execute the following

```bash
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

```bash
brew install python
```

Clone and compile ardupilot

```bash
git clone https://github.com/zenithaero/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

Don’t forget to regularly pull the latest changes

```bash
git pull origin master
```

Clone MAVProxy and add it to the PATH (assuming the shell is ZSH)

```bash
git clone https://github.com/zenithaero/MAVProxy.git
export MAV_PATH=/path/to/zenith/MAVProxy
echo "export PATH=$MAV_PATH/MAVProxy:$PATH" >> ~/.zshrc
```

And install the required dependencies

```bash
# dateutil is unsupported by some dependencies
pip uninstall python-dateutil
pip install wxPython gnureadline billiard numpy pyparsing
```

Follow the ardupilot build setup manual here. Here’s a summary of the steps to take

```bash
xcode-select --install
brew tap ardupilot/homebrew-px4
brew update
brew install genromfs gcc-arm-none-eabi gawk
pip install pyserial future empy pymavlink scipy
```

Install the OSX SDK headers (for version < Mojave)

```bash
open /Library/Developer/CommandLineTools/Packages/macOS_SDK_headers_for_macOS_10.14.pkg
```

# Building the sim

From ardupilot directory, enable the sitl. That step should only be executed once

```bash
cd ardupilot
./waf configure --board sitl
```

Compile the sitl

```bash
./waf build --target bin/arduplane
```

# Running the sim

Make the sim script executable

```bash
cd Zenith/Sim
chmod +x ./sim.py
```

Run the sitl on the default flight plan

```bash
./sim.py --f --speedup
```
