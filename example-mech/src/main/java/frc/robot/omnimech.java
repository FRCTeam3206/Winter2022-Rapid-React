// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.System;

import edu.wpi.first.wpilibj.DriverStation;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.drive.MecanumDrive;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class omnimech extends TimedRobot {
  private static final int kFrontLeftChannel = 0;
  private static final int kRearLeftChannel = 2;
  private static final int kFrontRightChannel = 1;
  private static final int kRearRightChannel = 3;

  private static final int kXBOXChannel = 0;

  // private MecanumDrive m_robotDrive;

  private XboxController m_stick;

  private VictorSP frontLeft = new VictorSP(kFrontLeftChannel);    // Front Left
  private VictorSP rearLeft = new VictorSP(kRearLeftChannel);       // Back Left
  private VictorSP frontRight = new VictorSP(kFrontRightChannel);  // Front Right
  private VictorSP rearRight = new VictorSP(kRearRightChannel);     // Back Right

  private static int debugMode = 0; // 0 for normal operation. Other numbers result in other functionality

  private double accelValues[] = {0, 0, 0};
  private static int FORWARD = 0;
  private static int STRAFE = 1;
  private static int ROTATE = 2;

  private static int ACCEL_CO = 8; // Accelleration coefficient
  private static double STICK_MOD = 0.5; // Artifical limit on input range
  private static double STICK_DEADZONE = 0.1; // xbox controller deadband

  private double stickTotal = 0;  // Total values of all stick inputs outside the deadband

  private double stick_LX = 0;
  private double stick_LY = 0;
  private double stick_RX = 0;

  private int fakeBool_Forward = 0;
  private int fakeBool_Strafe = 0;
  private int fakeBool_Rotate = 0;

  private double leftX_Fine = 0;
  private double leftY_Fine = 0;
  private double rightX_Fine = 0;

  private double powVals[] = {0, 0, 0};

  @Override
  public void robotInit() {
    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontLeft.setInverted(true);
    rearLeft.setInverted(true);

    //m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_stick = new XboxController(kXBOXChannel);

  }

  public double realValue(double a) {
    int fakeBool = (a < 0) ? 1 : 0;   // In C++, this is branchless. Here, we need an if-statement
    return a + -2 * a * fakeBool;     // If fakeBool is 1, the input value is negative  
  }

  public void debugHandle(int mode) {
    switch(mode) {
      case(1):
      // Motor check
      frontRight.set(0.1);
      frontLeft.set(0.1);
      rearRight.set(0.1);
      rearLeft.set(0.1);
      break;
      case(2):
        // Input check
        final double DBGSTICK = 0.1;
        double leftStick_X = m_stick.getLeftX() * DBGSTICK * -1;
        double leftStick_Y = m_stick.getLeftY() * DBGSTICK * -1;
        double rightStickX = m_stick.getRightX() * DBGSTICK;

        frontRight.set(leftStick_Y - leftStick_X - rightStickX);
        frontLeft.set(leftStick_Y + leftStick_X + rightStickX);
        rearRight.set(leftStick_Y + leftStick_X - rightStickX);
        rearLeft.set(leftStick_Y - leftStick_X + rightStickX);
      break;
      default:
        System.out.println("[DEBUG] Error: Incorrect debugMode value");
      return;
    }
  }

  protected double stickFine(double stickVal, double minVal, double maxVal, double lowerBound, double upperBound) {
    // Return the fine conversion value of a variable linear transformation
    // Credit: Simone on StackExchange: https://stats.stackexchange.com/a/178629
    return (upperBound - lowerBound) * ((stickVal - minVal) / (maxVal - minVal)) + lowerBound;
  }

  protected boolean checkDeadzone(double stickIn) {
    return (stickIn >= STICK_DEADZONE || stickIn <= STICK_DEADZONE * -1) ? true : false;
  }

  protected int incStickTotal(boolean isDirection) {
    return (isDirection == true) ? 1 : 0;
  }

  protected void reportWarning(String msg) {
    edu.wpi.first.wpilibj.DriverStation.reportWarning(msg, false);
  }

  @Override
  public void teleopPeriodic() {
    if(debugMode == 0) {
      stick_LX = m_stick.getLeftX() * STICK_MOD;
      stick_LY = m_stick.getLeftY() * STICK_MOD * -1;
      stick_RX = m_stick.getRightX() * STICK_MOD;
      
      boolean isForward = checkDeadzone(m_stick.getLeftY());
      boolean isStrafe = checkDeadzone(m_stick.getLeftX());
      boolean isRotate = checkDeadzone(m_stick.getRightX());

      stickTotal += incStickTotal(isForward);
      stickTotal += incStickTotal(isStrafe);
      stickTotal += incStickTotal(isRotate);
      
      leftY_Fine = stickFine(m_stick.getLeftY(), stickTotal * -1, stickTotal, -1, 1);   // Map -3 -> 3 to -1 -> 1
      leftX_Fine = stickFine(m_stick.getLeftX(), stickTotal * -1, stickTotal, -1, 1);   
      rightX_Fine = stickFine(m_stick.getRightX(), stickTotal * -1, stickTotal, -1, 1);
      
      /*
      edu.wpi.first.wpilibj.DriverStation.reportWarning("Left Stick Y Fine: " + leftY_Fine, false);
      edu.wpi.first.wpilibj.DriverStation.reportWarning("Left Stick X Fine: " + leftX_Fine, false);
      edu.wpi.first.wpilibj.DriverStation.reportWarning("Right Stick X Fine: " + rightX_Fine, false);
      */

      reportWarning("Left Stick Y Fine: " + leftY_Fine);
      reportWarning("Left Stick X Fine: " + leftX_Fine);
      reportWarning("Right Stick X Fine: " + rightX_Fine);

      // Accelleration formula so the motor speed isn't crazy right off the bat
        // This fomrula is simplified from: 1/k * input + (k - 1)/k * old motor pow = new motor pow
      accelValues[FORWARD] = (stick_LY + powVals[FORWARD] * ACCEL_CO - powVals[FORWARD]) / ACCEL_CO;
      accelValues[STRAFE] = (stick_LX + powVals[STRAFE] * ACCEL_CO - powVals[FORWARD]) / ACCEL_CO;
      accelValues[ROTATE] = (stick_RX + powVals[ROTATE] * ACCEL_CO - powVals[ROTATE]) / ACCEL_CO;
        
      // Automatically "break" when the driver isn't pressing anything
      if(fakeBool_Forward == 1) {
        powVals[FORWARD] = accelValues[FORWARD] * leftY_Fine;
      } else {
        powVals[FORWARD] = 0;
      }
      if(fakeBool_Strafe == 1) {
        powVals[STRAFE] = accelValues[STRAFE] * leftX_Fine;
      } else {
        powVals[STRAFE] = 0;
      }
      if(fakeBool_Rotate == 1) {
        powVals[ROTATE] = accelValues[ROTATE] * rightX_Fine;
      } else {
        powVals[ROTATE] = 0;
      }

      // Idea:
        // Use weighted averages to set the controller's fine tuning
          // Find whether a stick axis is greater / less than the deadzone
          // If so, add its value to the total & flip a boolean
          // Divide each stick's value by the total, then multiply the value with the previous result
          // Multiply the stick values by the weight value

      frontRight.set((powVals[FORWARD] - powVals[STRAFE] - powVals[ROTATE]) );
      frontLeft.set((powVals[FORWARD] + powVals[STRAFE] + powVals[ROTATE]) );
      rearRight.set((powVals[FORWARD] + powVals[STRAFE] - powVals[ROTATE]) );
      rearLeft.set((powVals[FORWARD] - powVals[STRAFE] + powVals[ROTATE]) );

      stickTotal = 0; // Reset stick total value so it doesn't skyrocket over time
    } else {
      debugHandle(debugMode);
    }
  }
}