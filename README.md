# TelsaCoilWinder

This repository contains the code and documentation needed to construct your own Telsa Coil Winder.    


## Organization

The repository is organized in the following fashion:
- 3DPrinterFiles:  3D printed files.
- AccelStepper_SketchV4:  Souce code for the Arduino Uno.  Accepts coil lenght and wire diameter as imput.   
- AccelStepper_SketchAlt: Souce Souce code for the Arduino Uno.  Accepts coil length, number of turns and wire diameter as input.
- Documentation:  Document describing the Tesla Coil Winder.  Explaination and parts list.
- EasyEDA: Source for custom PCB.
- Images:  Images of build
- JLCPCB:  Gerber file for PCB.
- Source files for the Nextion.

## Special thanks to:

Masculinity -  https://www.youtube.com/watch?v=HymLeTU4M8Q - This projects provided most of my inspiration for my build.

carl1961 - https://github.com/carl1961/Arduino-Nano-Tesla-Coil-Winder   He has been doing some great work.  Please make sure you take a look at his GitHub.  We have successfully beening sharing software and other components.

>[!NOTE]
> I would like to thanks carl1961 for all his work in refining the step sizes for the wire carraige and for his help in refining the wire diameter.  Also if your are using a lead screw with 2 starts carl1961 has already worked out the steps for the wire carriage.  

> [!CAUTION]
>This project is a work in progress, and well continue to change as I know more.  Picture shows AWG 26, 28, 32, and 34 wire testing.

## Implementations

There are two seperate implemention of the software. Make sure you pair the correct .ino file with the correct .hmi file.


### AccelStepper_sketchV4.ino & CoilWinderV4.HMI

This version use the coil length to calculate the number of turns and the estimated time for the build.


### AccelStepper_sketchAlt.ino & CoilWinderAlt.HMI

This version use the coil length, or the number of turns.  If coil length is specified, the number of turns is cacluated along with the estimated time.  If the number of turns is specified, the estimated coil length and the estimated time for the build is calculated.  


 Picture shows AWG 26, 28, 32, and 34 wire testing.

![Alt text](/Images/CoilWinder.jpg)


>[!NOTE]
If you have an questions or if you see a better way to have done something please open an issue and let me know. 