# Robotic-Arm
A robotic arm that mimics a user's arm gestures. It is purposed to help people with disabilities and meant for remote operations, and can also be used as a prototype in various other cases.

There are 2 parts to this project. The 1st part is the robotic arm itself, which has 5 DOF (5 servos / 5 "Degrees of Freedom") and uses an Arduino Nano, a Dedicated Servo Driver, and an NRF24L01 for recieving signals. The 2nd part is a gesture controller that straps onto the user's body. It has an Arduino Nano, an NRF24L01 module for transmitting data to the robotic arm, and 2 MPU6050's. The MPU6050s are velcroed onto the user's wrists, and then each direction of motion the user moves an arm in, causes a servo to move.

Programmed in `Arduino IDE`, models created in `Solidworks`, schematics created in `KiCad`.

Here is a very quick demo of the robotic arm being manipulated by the gesture controller that is strapped onto me. The full video is available [here](https://www.youtube.com/watch?v=RmKj0so72yQ). (Note: The white object in my hand is a switch to remotely power on/off the robotic arm.)
[![Link to youtube video](./Images/RoboticArmGif.gif)](https://www.youtube.com/watch?v=RmKj0so72yQ)

Additionally, I made a demo video showing all the joints in the robotic arm moving one by one (using the gesture controller), which is available for viewing [here](https://www.youtube.com/watch?v=eJz9jL7be58).

A picture of the Robotic Arm:
![RobotArmPic5](./Images/RobotArmPic0.jpg)

A picture of the Gesture Controller:
![RobotArmPic6](./Images/GesturePic.jpg)

## Schematics
![RobotArmSchematic](./Images/RoboticArmSchematic/RoboticArmSchematic.svg)

## Demos
Some demos showing how certain mechanisms in the robotic arm work.

| Gear Movement Demo | Backside of Gears |
| :---: | :---: |
| ![GearDemo](./Images/GearDemo.gif)  | ![BacksideOfGears](./Images/RobotArmPic9.jpg) |

<p align="center">
    <strong>Claw Demo</strong>
</p>

<p align="center">
  <img src="./Images/ClawDemo.gif"/>
</p>

## Other Images
Rendered pictures of the 3D model I created using my own models designed for 3D printing combined with the servos from [GrabCAD](https://grabcad.com/library). Made in real-world scale and was designed using Solidworks:
![RobotArmPic5](./Images/RobotArmPic8.JPG)

Render of gear mechanism underneath the lid.
![RobotArmPic5](./Images/RobotArmPic7.JPG)
