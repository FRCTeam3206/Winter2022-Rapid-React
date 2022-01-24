// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.System;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot {
  private static final int kFrontLeftChannel = 0;
  private static final int kRearLeftChannel = 2;
  private static final int kFrontRightChannel = 1;
  private static final int kRearRightChannel = 3;

  private static final int kXBOXChannel = 0;

  private MecanumDrive m_robotDrive;

  private XboxController m_stick;

  private double xbox_xAxisRStick = 0;
  private double xbox_yAxisRStick = 0;
  private double xbox_xAxisLStick = 0;
  private double xbox_yAxisLStick = 0;

  private boolean debugMode = true;

  @Override
  public void robotInit() {
    // 8 controllers for 8 motors
    VictorSP frontLeft = new VictorSP(kFrontLeftChannel);    // Front Left
    VictorSP rearLeft = new VictorSP(kRearLeftChannel);       // Back Left
    VictorSP frontRight = new VictorSP(kFrontRightChannel);  // Front Right
    VictorSP rearRight = new VictorSP(kRearRightChannel);     // Back Right

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontLeft.setInverted(true);
    frontRight.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_stick = new XboxController(kXBOXChannel);

  }

  @Override
  public void teleopPeriodic() {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
    xbox_xAxisRStick = m_stick.getRightX() * 0.1;   // Get stick axies and normalize
    xbox_yAxisRStick = m_stick.getRightY() * 0.1;     //-
    xbox_xAxisLStick = m_stick.getLeftX() * 0.1;      //-
    xbox_yAxisLStick = m_stick.getLeftY() * 0.1;    //-

    m_robotDrive.driveCartesian(xbox_xAxisLStick, xbox_yAxisLStick, xbox_xAxisRStick, 0.0);
    
    if(debugMode == true) {
      System.out.println("Right Stick X: " + m_stick.getRightX());
      System.out.println("Right Stick Y: " + m_stick.getRightY());
      System.out.println("---");
      System.out.println("Left Stick X: " + m_stick.getLeftX());
      System.out.println("Left Stick Y: " + m_stick.getLeftY());

      System.out.println("=========");

      System.out.println("Right Stick X (mod): " + xbox_xAxisRStick);
      System.out.println("Right Stick Y (mod): " + xbox_yAxisRStick);
      System.out.println("---");
      System.out.println("Left Stick X (mod): " + xbox_xAxisLStick);
      System.out.println("Left Stick Y (mod): " + xbox_yAxisLStick);
    }
  }
}
