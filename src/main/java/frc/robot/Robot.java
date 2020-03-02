/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.Servo;




/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {

  //Controller
  /**
   * Left Joystick is foward and backward, axis 1
   * Right Joystick is left and right, axis 4
   * RT is to shoot the balls, axis 3
   * RB push in to spin, release to stop, button 6
   */

  private final Joystick controller = new Joystick (0);

  VictorSPX LB = new VictorSPX(3);
  VictorSPX LF = new VictorSPX(2);
  VictorSPX RB = new VictorSPX(1);
  VictorSPX RF = new VictorSPX(0);
Servo servo = new Servo(0);
  private double startTime;

  //Color Sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  @Override
  public void robotInit() {
    LB.set(ControlMode.PercentOutput, 0);
    LF.set(ControlMode.PercentOutput, 0);
    RB.set(ControlMode.PercentOutput, 0);
    RF.set(ControlMode.PercentOutput, 0);
  }

 

  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    final double off = 0;
    final double turn_speed = 0.5;
    final double straight_speed = 0.2;
 
    final double time = Timer.getFPGATimestamp();

//LF positive and RF negative is forward
// current code allows robot to go in a square

  if (time - startTime < 1) {
 
    LF.set(ControlMode.PercentOutput, straight_speed);
    RF.set(ControlMode.PercentOutput, -straight_speed);
  }  

  else if (time - startTime > 1 && time - startTime < 2) {
    LF.set(ControlMode.PercentOutput, turn_speed);
    RF.set(ControlMode.PercentOutput, off);
  } 
  
  else if (time - startTime > 2 && time - startTime < 3) {
    LF.set(ControlMode.PercentOutput, straight_speed);

    RF.set(ControlMode.PercentOutput, -straight_speed);
  } 

  else if (time - startTime > 3 && time - startTime < 4) {
    LF.set(ControlMode.PercentOutput, turn_speed);

    RF.set(ControlMode.PercentOutput, off);
  } 

  else if (time - startTime > 4 && time - startTime < 5) {
    LF.set(ControlMode.PercentOutput, straight_speed);
    RF.set(ControlMode.PercentOutput, -straight_speed);
  } 
  
  else if (time - startTime > 5 && time - startTime < 6) {
    LF.set(ControlMode.PercentOutput, turn_speed);
    RF.set(ControlMode.PercentOutput, off);
  } 

  else if (time - startTime > 6 && time - startTime < 7) {
    LF.set(ControlMode.PercentOutput, straight_speed);
    RF.set(ControlMode.PercentOutput, -straight_speed);
  } 
  else {
      LB.set(ControlMode.PercentOutput, off);
      LF.set(ControlMode.PercentOutput, off);
      RB.set(ControlMode.PercentOutput, off);
      RF.set(ControlMode.PercentOutput, off);
  }


}
  

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    //Controller
    //final double off = 0;
    //final double on = 0.5;

  //attempted code to move joysticks, did not work 2/14
    double speed = 0;
    
    speed = controller.getRawAxis(3) - controller.getRawAxis(2);
    speed *= 0.2;
      // speed = controller.getRawAxis(3) * 0.6;
      // speed = -controller.getRawAxis(2) * 0.6;
    
    double turn = controller.getRawAxis(0) * 0.1;

    double left = speed + turn;
    double right = speed - turn;

    //Color
    final Color detectedColor = m_colorSensor.getColor();
    final double IR = m_colorSensor.getIR();

    LF.set (ControlMode.PercentOutput,left);
    LB.set (ControlMode.PercentOutput,left);
    RF.set (ControlMode.PercentOutput, -right);
    RB.set (ControlMode.PercentOutput,-right);
    
    servo.setAngle(0);
    // the idea here was to set motors to the left and right speed according
    //to joystick positions. did not work 2/14
    /*if (Math.abs(turn) < 0.005  || Math.abs(speed) < 0.005) {
      //turn = 0;
      //\speed = 0;
      left = speed + turn;
      right = speed - turn;
      LF.set (ControlMode.PercentOutput,left);
      LB.set (ControlMode.PercentOutput,left);
      RF.set (ControlMode.PercentOutput,-right);
      RB.set (ControlMode.PercentOutput,-right);

    }

    else {
      LF.set (ControlMode.PercentOutput,left);
      LB.set (ControlMode.PercentOutput,left);
      RF.set (ControlMode.PercentOutput,-right);
      RB.set (ControlMode.PercentOutput,-right);
    }*/

    System.out.println("this is the left value:"+left);
    System.out.println("this is the right value:"+right);
    System.out.println("this is the speed value:"+speed);
    System.out.println("this is the turn value:"+turn);

/*
    LF.set (ControlMode.PercentOutput,left);
    LB.set (ControlMode.PercentOutput,left);
    RF.set (ControlMode.PercentOutput,-right);
    RB.set (ControlMode.PercentOutput,-right);

*/



// mapping on the joysticks below. currently robot move forward with RB
// and A. B and LB makes robot go backwards

    // if (controller.getRawButton(1)){
    //   System.out.println("A");
    //   RF.set(ControlMode.PercentOutput,-on);
     
    
    // }
    
    // else { 
      
    //   RF.set(ControlMode.PercentOutput,off);
    
    // }
    
    
    // if (controller.getRawButton(2)){
    //   System.out.println("B");
    //   RB.set(ControlMode.PercentOutput,on);
    // }else{
    //   RB.set(ControlMode.PercentOutput,off);
    // }

    // if (controller.getRawButton(3)){
    //   System.out.println("X");
    // }
    // if (controller.getRawButton(4)){
    //   System.out.println("Y");
    // }
    // if (controller.getRawButton(5)){
    //   System.out.println("LB");
    //   LB.set(ControlMode.PercentOutput,-on);
    // }

    // else{
    //   LB.set(ControlMode.PercentOutput,off);

    // }
    // if (controller.getRawButton(6)){
    //   System.out.println("RB");
    //   LF.set(ControlMode.PercentOutput,on);

    // }
    
    // else{
    //   LF.set(ControlMode.PercentOutput,off);

    // }
    
    //Triggers
    /*
    if (controller.getAxis(Joystick.AxisType(4) )){
      System.out.println("LT");
    }
    if (controller.getRawAxis(3)){
      System.out.println("RT");
    }
    */

    
//display color readings from color sensor
  SmartDashboard.putNumber("Red", detectedColor.red);
  SmartDashboard.putNumber("Green", detectedColor.green);
  SmartDashboard.putNumber("Blue", detectedColor.blue);
  SmartDashboard.putNumber("IR", IR);

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
