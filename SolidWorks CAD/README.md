# SolidWorks 3D CAD Files  

## 🚀 Current development  
- Cuboid body with X-configuration arm
- Selection of landing gear
- Selection of material (FEA & Granta Edupack etc.) 

## 🧐 Changelog  
13 Feb 21 — Add files via upload  

## 📃 File extensions  
**`.SLDPRT`** &emsp; SOLIDWORKS Part File  
**`.SLDASM`** &emsp; SOLIDWORKS Assembly File  
**`.STL   `** &emsp;  File type compatible for 3D additive manufacturing  

*I'm using SOLIDWORKS 2020 Student Edition, so if you are using a more recent version it should be backwards compatible.*  

## ⚙ Contents  

### `Assembly1.SLDASM` 
 
<table>
  <tr align="center">
    <th>Component filename</th>
    <th>Description</th>
    <th>Quantity</th>
  </tr>
  <tr align="center">
    <td>arm.SLDPRT</td>
    <td>X-configuration arm</td>
    <td>1</td>
  </tr>
  <tr align="center">
    <td>blade.SLDPRT</td>
    <td>Rotor blade</td>
    <td>4</td>
  </tr>
  <tr align="center">
    <td>body.SLDPRT</td>
    <td>Hexagonal body</td>
    <td>1</td>
  </tr>
  <tr align="center">
    <td>gimbal1.SLDPRT</td>
    <td>Gimbal subpart</td>
    <td>1</td>
  </tr>
    <tr align="center">
    <td>gimbal2.SLDPRT</td>
    <td>Gimbal subpart</td>
    <td>1</td>
  </tr>
    <tr align="center">
    <td>gimbal3.SLDPRT</td>
    <td>Gimbal subpart</td>
    <td>1</td>
  </tr>
    <tr align="center">
    <td>gimbal4.SLDPRT</td>
    <td>Gimbal subpart</td>
    <td>1</td>
  </tr>
    <tr align="center">
    <td>camera.SLDPRT</td>
    <td>Camera</td>
    <td>1</td>
  </tr>
</table>

Associated .STL file: [concept1.STL](https://github.com/kaizer222/FEEG2001_DroneAssignment/blob/main/SolidWorks%20CAD/concept1.STL)  
*The gimbal model used in this assembly is inaccurate and oversimplified, please refer to `ggimbal_assem.SLDASM` instead.*

### `Assembly2.SLDASM` 
 
<table>
  <tr align="center">
    <th>Component filename</th>
    <th>Description</th>
    <th>Quantity</th>
  </tr>
  <tr align="center">
    <td>arm2.SLDPRT</td>
    <td>H-configuration arm</td>
    <td>1</td>
  </tr>
  <tr align="center">
    <td>blade.SLDPRT</td>
    <td>Rotor blade</td>
    <td>4</td>
  </tr>
  <tr align="center">
    <td>body2.SLDPRT</td>
    <td>Cuboid body</td>
    <td>1</td>
  </tr>
  <tr align="center">
    <td>angle_bracket_15x15x10.SLDPRT</td>
    <td>Angle bracket</td>
    <td>2</td>
</table>

Associated .STL file: [concept2.STL](https://github.com/kaizer222/FEEG2001_DroneAssignment/blob/main/SolidWorks%20CAD/concept2.STL)  

### `servo_assem.SLDASM`  

<table>
  <tr align="center">
    <th>Component filename</th>
    <th>Description</th>
    <th>Quantity</th>
  </tr>
  <tr align="center">
    <td>servo_body.SLDPRT</td>
    <td>Servo body</td>
    <td>1</td>
  </tr>
  <tr align="center">
    <td>servo_blade.SLDPRT</td>
    <td>Servo blade</td>
    <td>1</td>
  </tr>
</table>

Associated .STL file: [servo.STL](https://github.com/kaizer222/FEEG2001_DroneAssignment/blob/main/SolidWorks%20CAD/servo.STL)  
*The modelling of the servo itself is not important, but it helps when modelling the servo housing for the gimbal as in `ggimbal_assem.SLDASM`.*  

### `ggimbal_assem.SLDASM`  

<table>
  <tr align="center">
    <th>Component filename</th>
    <th>Description</th>
    <th>Quantity</th>
  </tr>
  <tr align="center">
    <td>servo_body.SLDPRT</td>
    <td>Servo body</td>
    <td>3</td>
  </tr>
  <tr align="center">
    <td>servo_blade.SLDPRT</td>
    <td>Servo blade</td>
    <td>3</td>
  <tr align="center">
    <td>camera_scaled.SLDPRT</td>
    <td>Just a fancier camera 📸</td>
    <td>1</td>
  </tr>
  <tr align="center">
    <td>servo_bolt.SLDPRT</td>
    <td>Servo bolt (short)</td>
    <td>2</td>
  </tr>
  <tr align="center">
    <td>servo_bolt_long.SLDPRT</td>
    <td>Servo bolt (long)</td>
    <td>4</td>
  </tr>
  <tr align="center">
    <td>servo_nut.SLDPRT</td>
    <td>Servo nut</td>
    <td>6</td>
  </tr>
  <tr align="center">
    <td>servo_housing1.SLDPRT</td>
    <td>Servo housing (roll direction)</td>
    <td>1</td>
  </tr>
  <tr align="center">
    <td>servo_housing2.SLDPRT</td>
    <td>Servo housing (pitch direction)</td>
    <td>1</td>
  </tr>
</table>

Associated .STL file: [gimbal.STL](https://github.com/kaizer222/FEEG2001_DroneAssignment/blob/main/SolidWorks%20CAD/gimbal.STL)  
*Three degrees of rotation freedom (DOF-3R), each with π radians.*

---
