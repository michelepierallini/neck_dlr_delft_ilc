# Tendon actuation controller

Project to test out different controllers to achieve compliant behaviour for tendon actuation.

## Getting started
Install two extenal libraries:
- [Dynamixel](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
- [Arduino](https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE)

## Descrption 
- [arduino](arduino) : Arduino library to handle all the device we have mounted in the platform.
- [CAD](CAD) : Folder thats contains the some of the cad file. The overall project can be found in [here](https://github.com/DLR-RM/TendonDrivenContinuum)
- [pyhton](pyhton) : Folder thats contains the code to move the neck and also execute the ilc algorithm

## Test and Deploy
To run the code one can just write the following code 
```bash
python tor_contr_main.py
```
## Setup 
In the [config](python/config/config_controller1.py) file set the correct port of the motors. This can be done using the Arduino IDE.
Additionally set the pricipal command into the [main](python/tor_contr_main.py) code here:
```python
servo = dynamixel(ID_structure,
                    descriptive_device_name = "XM430 motor",
                    series_name = ["xm", "xm", "xm", "xm"],
                    baudrate = 1000000,
                    port_name = "COM5")
```
## Utility on the code 
To run a simple feedforward action one can use [pure_feedforward.py](python/pure_feedforward.py). 
To just stream data and eventually pre-tension the tendons one can use [stream_data.py](python/stream_data.py)
[utility.py](python/utility.py) and [utils_data.py](python/utils_data.py) contain utilities function to plot data or compute enetities
***

## License
For open source projects, say how it is licensed.

## Project status
This project is ongoing. Part of the work has been submitted to RA-L. 
