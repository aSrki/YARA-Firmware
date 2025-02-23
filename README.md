## Project YARA - Firmware

The main idea of YARA firmware is for it to be as simple and as understandable as possible. It needs to support basic features such as communication and motor control in order to achieve workable robotic system.

Firmware of YARA robot was developed using STM32 NUCLEO F303K8 developement board. Code was generated/written in STMCubeIDE using HAL library in order to make it as easy to read and understand as possible. 
Communication between PC and NUCLEO board is accomplished using USB/UART protocol. 

<p align="center">
  <img src="https://github.com/aSrki/YARA-Firmware/blob/main/images/NUCLEO-F303K8.jpg" width="400"/>
</p>

Stepper motor drivers used in this project are TMC2209. They were chosen because of their small footprint, "silent driver" technology and easu-of-use. 
They are controled using STEP and DIR pins because of easy of use, although UART communication way of control should be a better solution. (Maybe TODO for later?)

<p align="center">
  <img src="https://github.com/aSrki/YARA-Firmware/blob/main/images/TMC2209.jpg" width="400"/>
</p>
