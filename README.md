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

carl1961 - https://github.com/carl1961/Arduino-Nano-Tesla-Coil-Winder   Who has beening using some of the code from my project.

>[!NOTE]
> I would like to thanks carl1961 for all his work in refining the step size for the wire carraige and for his help in refining the wire diameter.

> [!CAUTION]
>This project is a work in progress, and well continue to change as I know more.  Picture shows AWG 26, 28, 32, and 34 wire testing.

## Implementations

There are two seperate implemention of the software. Make sure you pair the correct .ino file with the correct .hmi file.


### AccelStepper_sketchV4.ino & CoilWinderV4.HMI

This version use the coil length to calculate the number of turns and the estimated time of the build.


### AccelStepper_sketchAlt.ino & CoilWinderAlt.HMI

This version use the coil length, or the number of turns.  If coil length is specified the number of turns is cacluated along with the estimated time.  If the number of turns was specified the estimated time for the build is calculated.  



![Alt text](/Images/CoilWinder.jpg)


