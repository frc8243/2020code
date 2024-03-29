/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {

  //button and axis indices for gamepad
  private final int LEFT_TRIGGER = 2;
  private final int RIGHT_TRIGGER = 3;
  private final int LEFT_BUMPER = 5;
  private final int RIGHT_BUMPER = 6;
  private final int LEFT_STICK_X = 0;
  private final int LEFT_STICK_Y = 1;
  private final int RIGHT_STICK_X = 4;
  private final int RIGHT_STICK_Y = 5;
  private final int A_BUTTON = 1;
  private final int B_BUTTON = 2;
  private final int X_BUTTON = 3;
  private final int Y_BUTTON = 4;
  private final int START_BUTTON = 8;
  private final int BACK_BUTTON = 7;

  //Controller
  /**
   * Left Joystick is foward and backward, axis 1
   * Right Joystick is left and right, axis 4
   * RT is to shoot the balls, axis 3
   * RB push in to spin, release to stop, button 6
   */

  //TODO #1: Add second joystick for controlling other robot mechanisms
  //TODO #2: Create enum or variables for button and axis mappings

  private final Joystick controller = new Joystick (0); // for controlling chasis / driving

  /*
    Using `WPI_VictorSPX` instead of `VictorSPX` because it's compatible
    with `SpeedControllerGroup`. See https://www.chiefdelphi.com/t/can-weirdness/335403/9
    for details.
  */
  WPI_VictorSPX LB = new WPI_VictorSPX(3);
  WPI_VictorSPX LF = new WPI_VictorSPX(2);
  WPI_VictorSPX RB = new WPI_VictorSPX(1);
  WPI_VictorSPX RF = new WPI_VictorSPX(0);

  //https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/SpeedControllerGroup.html
  SpeedControllerGroup leftGroup = new SpeedControllerGroup(LB, LF);
  SpeedControllerGroup rightGroup = new SpeedControllerGroup(RB, RF);

  //https://youtu.be/g-dgdWVO5u8?t=699
  //https://docs.wpilib.org/en/latest/docs/software/actuators/wpi-drive-classes.html#multi-motor-differentialdrive-with-speedcontrollergroups
  DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

  //https://docs.wpilib.org/en/latest/docs/software/actuators/servos.html#constructing-a-servo-object
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
    //Uncommment if right side motors need to run in opposite direction (a.k.a. inverted)
    drive.setRightSideInverted(true);

    //stop robot
    drive.arcadeDrive(0, 0);

    //reset servo
    servo.setAngle(0);

    //TODO #10: Start camera capture
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

    //TODO #3: Replace motor controller calls with `DifferentialDrive.arcadeDrive`
    //TODO #4: Program to drive robot across line
    //TODO #5: Use encoder to measure how far robot has travelled
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
    /*
      Team uses "racing game" controls for driving
      right trigger - accelerate
      left trigger  - deaccelerate
      left joystick - steering

      Speed is calculate as the difference between acceleration and deacceleration

    */
    double speed = controller.getRawAxis(RIGHT_TRIGGER) - controller.getRawAxis(LEFT_TRIGGER);    
    double turn = controller.getRawAxis(LEFT_STICK_X);

    //Color
    final Color detectedColor = m_colorSensor.getColor();
    final double IR = m_colorSensor.getIR();

    //updates motor controller output
    drive.arcadeDrive(speed, turn);

    //TODO #6: Display values in shuffleboard
    SmartDashboard.putNumber("LB", LB.getMotorOutputPercent());
    SmartDashboard.putNumber("LF", LB.getMotorOutputPercent());
    SmartDashboard.putNumber("RB", LB.getMotorOutputPercent());
    SmartDashboard.putNumber("RF", LB.getMotorOutputPercent());

    //display color readings from color sensor
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    //TODO #11: Program buttons to control servo
    if(controller.getRawButton(A_BUTTON)){
      servo.set(1);
    }

    else if (controller.getRawButton(B_BUTTON)){
      servo.set(0);
    }

    //TODO #7: Read confidence reading from color sensor
    //TODO #8: Use confidence reading and other values to determine color
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
