# Loadcell arduino # M4
loadcell_arduino_config =   {   "name":"Loadcell arduino",
                                "baudrate":115200,
                                "port":"COM3"
                            }   

# Arduino Motor 3 
loadcell_arduino_M3 =  {    "name":"Loadcell arduino motor 3",
                                    "baudrate":115200,
                                    "port":"COM8"
                                }


# Arduino Motor 1 
loadcell_arduino_M1 =  {    "name":"Loadcell arduino motor 1",
                                    "baudrate":115200,
                                    "port":"COM4"
                                }


# Arduino Motor 2 
loadcell_arduino_M2 =  {    "name":"Loadcell arduino motor 2",
                                    "baudrate":115200,
                                    "port":"COM7"
                                }

# Arduino IMU
arduino_IMU =  {    "name":"Arduino IMU",
                                    "baudrate":9600,
                                    "port":"COM6"
                                }

# Dynamixel
dynamixel_config =  {   "name":"Force controlled dynamixel",
                        "baudrate":1000000,
                        "port":"COM5",
                        "series":"xm",
                        "ID":4
                    }

# Plotjuggler
plotjuggler_config =    {  "port":9872
                        }


# Physical parameters
#force_conversion = 1/(750/9.8) #bits2N_loadcell *N_loadcell2Nm_motor 


# Control parameters
#kp = 0
#kd = 0
#ki = 0

#increment = 0.0001

