// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.System;
import java.util.*;       // This is temporary, I'll replace it with what exactly is used later

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

  private double xbox_xAxisRStickMOD = 0;
  private double xbox_yAxisRStickMOD = 0;
  private double xbox_xAxisLStickMOD = 0;
  private double xbox_yAxisLStickMOD = 0;

  private static double xbox_xAxisRStickMAX = 0; // TODO; Fill these in
  private static double xbox_yAxisRStickMAX = 0;  //-
  
  private static double xbox_xAxisLStickMAX = 0;  //-
  private static double xbox_yAxisLStickMAX = 0;  //-

  private static double xbox_xAxisRStickMIN = 0;  //-
  private static double xbox_yAxisRStickMIN = 0;  //-
  
  private static double xbox_xAxisLStickMIN = 0;  //-
  private static double xbox_yAxisLStickMIN = 0;  //-

  private double xbox_rStickAngle = 0;  // These will be measured in degrees
  private double xbox_lStickAngle = 0;

  private double xbox_lStickNormalX = 0;
  private double xbox_lStickNormalY = 0;
  private double xbox_rStickNormalX = 0;
  private double xbox_rStickNormalY = 0;

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

  // Function to normalize inputs between -1 and 1
  // Thanks to Simone on SE: https://stats.stackexchange.com/a/178629
  public int normalizeStickVals(int i, int max, int min) {
    return 2 * (i - min) / (max - min) - 1;
  }

  public double findMinOrMax(double a, double b, boolean minOrMax) { // Find the min/max of a value. True = find max, False = find min
    switch(minOrMax) {
      case(true):
          if(a > b) {
            return a;
          } else {
            return b;
          }
        break;
      case(false):
          if(a < b) {
            return a;
          } else {
            return b;
          }
        break;
      default:
        System.out.println("Please use a valid min/max option. True = max, false = min");
        break;
    }
  }

  public double getHypo(double a, double b) {
    // Pythagorean theorem: a^2 + b^2 = c^2
      // Or: c = sqrt( a^2 + b^2 )
    return Math.sqrt(Math.pow(a) + Math.pow(b));
  }

  @Override
  public void teleopPeriodic() {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
    xbox_xAxisRStick = m_stick.getRightX();   // Get stick axies
    xbox_yAxisRStick = m_stick.getRightY();     //-
    xbox_xAxisLStick = m_stick.getLeftX();      //-
    xbox_yAxisLStick = m_stick.getLeftY();      //-

    // Get max and min for each stick
    xbox_xAxisRStickMAX = findMinOrMax(xbox_xAxisRStickMAX, xbox_xAxisRStick, true);
    xbox_xAxisRStickMIN = findMinOrMax(xbox_xAxisRStickMIN, xbox_xAxisRStick, false);
    xbox_yAxisRStickMAX = findMinOrMax(xbox_yAxisRStickMAX, xbox_yAxisRStick, true);
    xbox_yAxisRStickMIN = findMinOrMax(xbox_yAxisRStickMIN, xbox_yAxisRStick, false);

    xbox_xAxisLStickMAX = findMinOrMax(xbox_xAxisLStickMAX, xbox_xAxisLStick, true);
    xbox_xAxisLStickMIN = findMinOrMax(xbox_xAxisLStickMIN, xbox_xAxisLStick, false);
    xbox_yAxisLStickMAX = findMinOrMax(xbox_yAxisLStickMAX, xbox_yAxisLStick, true);
    xbox_yAxisLStickMIN = findMinOrMax(xbox_yAxisLStickMIN, xbox_yAxisLStick, false);

    // Weigh values down
    xbox_xAxisRStickMOD = xbox_xAxisRStick * 0.1;
    xbox_yAxisRStickMOD = xbox_yAxisRStick * 0.1;
    xbox_xAxisLStickMOD = xbox_xAxisLStick * 0.1;
    xbox_yAxisLStickMOD = xbox_yAxisLStick * 0.1;
    
    //m_robotDrive.driveCartesian(xbox_xAxisLStickMOD, xbox_yAxisLStickMOD, xbox_xAxisRStickMOD, 0.0);
    
    // Normalize stick values between -1 & 1
    xbox_rStickNormalX = normalizeStickVals(xbox_xAxisRStick, xbox_xAxisRStickMAX, xbox_xAxisRStickMIN);
    xbox_rStickNormalY = normalizeStickVals(xbox_yAxisRStick, xbox_yAxisRStickMAX, xbox_yAxisRStickMIN);
    xbox_lStickNormalX = normalizeStickVals(xbox_xAxisLStick, xbox_xAxisLStickMAX, xbox_xAxisLStickMIN);
    xbox_lStickNormalY = normalizeStickVals(xbox_yAxisLStick, xbox_yAxisLStickMAX, xbox_yAxisLStickMIN);

    // Calculate the angle of the stick
    double rStick_hypo = getHypo(xbox_rStickNormalX, xbox_rStickNormalY);
    double rStick_sideA = xbox_rStickNormalX / rStick_hypo;
    double rStick_sideB = xbox_rStickNormalY / rStick_hypo;
    xbox_rStickAngle = Math.atan(rStick_sideB / rStick_sideA);

    double lStick_hypo = getHypo(xbox_lStickNormalX, xbox_lStickNormalY);
    double lStick_sideA = xbox_lStickNormalX / lStick_hypo;
    double lStick_sideB = xbox_lStickNormalY / lStick_hypo;
    xbox_lStickAngle = Math.atan(lStick_sideB / lStick_sideA);

    // TODO:
      // Calculate the stick's angle via trig
      // Use the angle to transforms on motor power
        // (1, 0) Is strafe right
        // (-1, 0) Is strafe left
        // (0, 1) Is drive forward
        // (0, -1) Is drive backward

        // (sqr2/2, sqr2/2) Is strafe diagonally up-right
        // (-sqr2/2, sqr2/2) Is strafe diagonally up-left
        // (-sqr2/2, -sqr2/2) Is strafe diagonally down-left
        // (sqr2/2, -sqr2/2) Is strafe diagonally down-right

        // There's more but I'm lazy and tired to describe them rn

    if(debugMode == true) {
      System.out.println("Right Stick X: " + xbox_xAxisRStick);
      System.out.println("Right Stick Y: " + xbox_yAxisRStick);
      System.out.println("---");
      System.out.println("Left Stick X: " + xbox_xAxisLStick);
      System.out.println("Left Stick Y: " + xbox_yAxisLStick);

      System.out.println("=========");

      System.out.println("Right Stick X (mod): " + xbox_xAxisRStickMOD);
      System.out.println("Right Stick Y (mod): " + xbox_yAxisRStickMOD);
      System.out.println("---");
      System.out.println("Left Stick X (mod): " + xbox_xAxisLStickMOD);
      System.out.println("Left Stick Y (mod): " + xbox_yAxisLStickMOD);

      System.out.println("|||||||||");

      System.out.println("Right Stick X (normal): " + xbox_rStickNormalX);
      System.out.println("Right Stick Y (normal): " + xbox_rStickNormalY);
      System.out.println("---");
      System.out.println("Left Stick X (normal): " + xbox_lStickNormalX);
      System.out.println("Left Stick Y (normal): " + xbox_lStickNormalY);

      System.out.println("/////////");

      System.out.println("Right Stick Angle: " + xbox_rStickAngle);
      System.out.println("Left Stick Angle: " + xbox_lStickAngle);
    }
  }
}
