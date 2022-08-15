// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */

public class Robot extends TimedRobot {
  private DifferentialDrive robot;
  private GenericHID joystick;
  private static final int leftLeadDeviceID = 1; 
  private static final int rightLeadDeviceID = 3;
  private static final int leftFollowDeviceID = 2;
  private static final int rightFollowDeviceID = 4;
  private CANSparkMax leftLeadMotor;
  private CANSparkMax rightLeadMotor;
  private CANSparkMax leftFollowMotor;
  private CANSparkMax rightFollowMotor;
  private SlewRateLimiter accelLimit=new SlewRateLimiter(.8);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    leftLeadMotor = new CANSparkMax(leftLeadDeviceID, MotorType.kBrushed);
    leftFollowMotor = new CANSparkMax(leftFollowDeviceID, MotorType.kBrushed);
    rightLeadMotor = new CANSparkMax(rightLeadDeviceID, MotorType.kBrushed);
    rightFollowMotor = new CANSparkMax(rightFollowDeviceID, MotorType.kBrushed);

    leftFollowMotor.follow(leftLeadMotor);
    rightFollowMotor.follow(rightLeadMotor);

    leftLeadMotor.setInverted(true);

    robot = new DifferentialDrive(leftLeadMotor, rightLeadMotor);
    joystick = new GenericHID(0);

    CameraServer.startAutomaticCapture();
  }

  double currRight=0;
  double currLeft=0;
  
  @Override
  public void teleopPeriodic() {
    double forward=accelLimit.calculate(joystick.getRawAxis(1)/1.5);
    if(Math.abs(joystick.getRawAxis(1))<.5){
      forward=0;
    }
    robot.arcadeDrive(forward, -joystick.getRawAxis(2)/1.5);
  }
  
  private double cut(double val){
    if(val<-1)return -1;
    if(val>1)return 1;
    return val;
  }
}
