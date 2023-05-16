# 3D LiDAR scanner with stepper motors

This repository demonstrates a homemade 3D LiDAR scanner made out of TF-Mini-S LiDAR sensor and 2 stepper motors. This scanner can produce 3D cloud point of spaces.
The LiDAR sensor is fixed on one stepper arm with the help of a 3D printed bracket. This stepper is fixed to the second stepper arm through another bracket. 

![Picture of the 3D LiDAR scanner](scannerLidar-stepper-02-crop.jpg) ![Example of 3D scan vizualisation using Processing](scan3D-exemple.jpg)

## Files in the repository
- MWE-scanner_stepper.ino : file to be uploaded to the Arduino board
- stepperScaneer-serial2CSV.py : script to save scan data into a CSV file
- LidarScannerVisualizer.pde : [Processing](https://processing.org/) script receiving serial command and displaying 3D view of the scene based on the xyz coordinates comming from the scanner

## Arduino Script

Library needed : 
- TFMPlus : [https://github.com/budryerson/TFMini-Plus][(https://github.com/budryerson/TFMini-Plus)
- SoftwareSerial : included into Arduino framework

Material used
https://blog.protoneer.co.nz/arduino-cnc-shield/



## Scanner serial protocol
The scanner can be control through serial protocol with the following commands : 

- s : perform 2D scan
- y : perform 1D scan
- i : display scanner setup info
- v : followed by an integer value sets the vertical step
- h : followed by an integer value sets the horizontal step
- p : scan reboot
   
Once the scan is launched (through s or y), it performs the XYZ coordinates conversion and ouputs all of the data throught serial port in the following manner : 

`x y z h_idx v_idx distance `

- h_idx : horizontal index of the scan
- v_idx vertical index of the scan
- distance : data coming from TF-Mini-s LiDAR sensor
- x : x coordinate in 3D space
- y : y coordinate in 3D space
- z : z coordinate in 3D space



## stepperScaneer-serial2CSV.py 


## License
 © 2022 – CLICK - Université de Mons

3D LiDAR scanner with stepper motors – CLICK - UMONS (Loïc Reboursière) is free software: you can redistribute it and/or modify it under the terms of the Apache License Version 2.0. This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the Apache License Version 2.0 for more details.
You should have received a copy of the Apache License Version 2.0 along with this program.  If not, see http://www.apache.org/licenses/
Each use of this software must be attributed to University of MONS – CLICK (Loïc Reboursière).
Any other additional authorizations may be asked to avre@umons.ac.be.

## Legal Notices
This work was produced as part of the FEDER Digistorm project, co-financed by the European Union and the Wallonia Region.

![Logo FEDER-FSE](https://www.enmieux.be/sites/default/files/assets/media-files/signatures/vignette_FEDER%2Bwallonie.png)