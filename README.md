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


***

## License
For open source projects, say how it is licensed.

## Project status
This project is ongoing. Part of the work has been submitted to RA-L. 
