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
  private double STICK_MOD = 0.5; // Artifical limit on input range
  private static double STICK_DEADZONE = 0.1; // xbox controller deadband

  private double stickTotal = 0;  // Total values of all stick inputs outside the deadband

  private double stick_LX = 0;
  private double stick_LY = 0;
  private double stick_RX = 0;

  private double leftX_Fine = 0;
  private double leftY_Fine = 0;
  private double rightX_Fine = 0;

  private double powVals[] = {0, 0, 0};

  private boolean aPressed = false;
  private boolean yPressed = false;

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
      case(3): // Test FRC 6624's mecanum code
        mechDrive6624();
      break;
      default:
        String msg = "[DEBUG] Error: Incorrect debugMode value";
        System.out.println(msg);
        reportWarning(msg);
        endCompetition();
      return;
    }
  }

  protected double stickFine(double stickVal, double minVal, double maxVal, double lowerBound, double upperBound) {
    // Return the fine conversion value of a variable linear transformation
    // Credit: Simone on StackExchange: https://stats.stackexchange.com/a/178629
    double result = (upperBound - lowerBound) * ((stickVal - minVal) / (maxVal - minVal)) + lowerBound;
    return (result > upperBound || result < lowerBound ||  Double.isNaN(result) == true) ? 0 : result; // Check if the values aren't crazy. IDK how to filter out NaN without also getting werid numbers
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

  protected double setPowVals(double accel, double fine, boolean isDirection) {
    // Based on:
      /*
      if(isForward = true) {
        powVals[FORWARD] = accelValues[FORWARD] * leftY_Fine;
      } else {
        powVals[FORWARD] = 0;
      }
      if(isStrafe == true) {
        powVals[STRAFE] = accelValues[STRAFE] * leftX_Fine;
      } else {
        powVals[STRAFE] = 0;
      }
      if(isRotate == true) {
        powVals[ROTATE] = accelValues[ROTATE] * rightX_Fine;
      } else {
      powVals[ROTATE] = 0;
      }
      */
    
    if(isDirection == true) {
      //return accel * fine;
      return fine * STICK_MOD;
    } else {
      return 0;
    }
  }

  protected double setAccelVals(double stickVal, double lastPow) {
    // Based on:
      // Accelleration formula so the motor speed isn't crazy right off the bat
      // This fomrula is simplified from: 1/k * input + (k - 1)/k * old motor pow = new motor pow
      /*
      accelValues[FORWARD] = (stick_LY + powVals[FORWARD] * ACCEL_CO - powVals[FORWARD]) / ACCEL_CO;
      accelValues[STRAFE] = (stick_LX + powVals[STRAFE] * ACCEL_CO - powVals[FORWARD]) / ACCEL_CO;
      accelValues[ROTATE] = (stick_RX + powVals[ROTATE] * ACCEL_CO - powVals[ROTATE]) / ACCEL_CO;
      */

    return (stickVal + lastPow * ACCEL_CO - lastPow) / ACCEL_CO;
  }

  protected double setMechanumSpeed(double forward, double strafe, double rotate, int wheel) {
    // Based on:
      /*
      frontRight.set(powVals[FORWARD] - powVals[STRAFE] - powVals[ROTATE]);
      frontLeft.set(powVals[FORWARD] + powVals[STRAFE] + powVals[ROTATE]);
      rearRight.set(powVals[FORWARD] + powVals[STRAFE] - powVals[ROTATE]);
      rearLeft.set(powVals[FORWARD] - powVals[STRAFE] + powVals[ROTATE]);
      */
    
    switch(wheel) {
      case(0):  // Front right wheel
        return forward - strafe - rotate;
      case(1): // Front left wheel
        return forward + strafe + rotate;
      case(2): // Back right wheel
        return forward + strafe - rotate;
      case(3): // Back left wheel
        return forward - strafe + rotate;
      default:
        reportWarning("Invalid wheel choice in func 'setMechanumSpeed'. Use a number 0-4");
        return 0;
    }
  }

  protected void incSpeed() {
    if(m_stick.getAButton() && aPressed != true) {
      if(STICK_MOD + 0.1 <= 1) {
        STICK_MOD += 0.1;
      }
        aPressed = true;
    }
    if(m_stick.getAButton() == false) {
      aPressed = false;
    }

  }

  protected void decSpeed() {
    if(m_stick.getYButton() && yPressed != true) {
      if(STICK_MOD - 0.1 >= 0.4) {
        STICK_MOD -= 0.1;
      }
      yPressed = true;
    }
    if(m_stick.getYButton() == false) {
      yPressed = false;
    }
  }

  protected double calcAngle(double x, double y) {
    double r = Math.sqrt((Math.pow(x, 2) + Math.pow(y, 2)));
    return Math.acos(x / r);
  }

  protected double calcTransPower(double x, double y) {
    int isX = (checkDeadzone(x)) ? 1 : 0;
    int isY = (checkDeadzone(y)) ? 1 : 0;

    int stickTotal = isX + isY;
    double sticks = x * isX + y * isY;

    return stickFine(sticks, stickTotal * -1, stickTotal, -1, 1);
  }

  protected void experimentalDrive(double translationAngle, double translationPower, double turnPower) {
    double ADPower = translationPower * Math.sqrt(2) * 0.5 * (Math.sin(translationAngle) + Math.cos(translationAngle));
    double BCPower = translationPower * Math.sqrt(2) * 0.5 * (Math.sin(translationAngle) - Math.cos(translationAngle));
  
    double turningScale = Math.max(Math.abs(ADPower + turnPower), Math.abs(ADPower - turnPower));
    turningScale = Math.max(turningScale, Math.max(Math.abs(BCPower + turnPower), Math.abs(BCPower - turnPower)));

    if(Math.abs(turningScale) > 1) {
      turningScale = 1;
    }

    frontLeft.set(ADPower);
    rearLeft.set(BCPower);
    frontRight.set(BCPower);
    rearRight.set(ADPower);
  }

  protected void mechDrive6624() {
    double x = m_stick.getLeftX();
    double y = m_stick.getLeftY();
    double z = m_stick.getRightX();

    double angle = calcAngle(x, y);
    double transPower = calcTransPower(x, y);

    experimentalDrive(angle, transPower, z);
  }

  protected void mechDriveCRW() {
    // Idea:
        // Use weighted averages to set the controller's fine tuning
          // Find whether a stick axis is greater / less than the deadzone
          // If so, add its value to the total & flip a boolean
          // Divide each stick's value by the total, then multiply the value with the previous result
          // Multiply the stick values by the weight value
    
    stick_LX = m_stick.getLeftX() * STICK_MOD;
    stick_LY = m_stick.getLeftY() * STICK_MOD;
    stick_RX = m_stick.getRightX() * STICK_MOD;
          
    boolean isForward = checkDeadzone(m_stick.getLeftY());
    boolean isStrafe = checkDeadzone(m_stick.getLeftX());
    boolean isRotate = checkDeadzone(m_stick.getRightX());
    
    stickTotal += incStickTotal(isForward);
    stickTotal += incStickTotal(isStrafe);
    stickTotal += incStickTotal(isRotate);
          
    // These currently can return numbers like NAN, -INFINITY and INFINITY. It doesn't break driving, but it's good to know
    leftY_Fine = stickFine(m_stick.getLeftY(), stickTotal * -1, stickTotal, -1, 1);   // Map anywhere between -3 & 3, -2 & 2, -1 & 1 to -1 & 1
    leftX_Fine = stickFine(m_stick.getLeftX(), stickTotal * -1, stickTotal, -1, 1);   
    rightX_Fine = stickFine(m_stick.getRightX(), stickTotal * -1, stickTotal, -1, 1);
          
    accelValues[FORWARD] = setAccelVals(stick_LY, powVals[FORWARD]);
    accelValues[STRAFE] = setAccelVals(stick_LX, powVals[STRAFE]);
    accelValues[ROTATE] = setAccelVals(stick_RX, powVals[ROTATE]);
    
    powVals[FORWARD] = setPowVals(accelValues[FORWARD], leftY_Fine, isForward) * -1;
    powVals[STRAFE] = setPowVals(accelValues[STRAFE], leftX_Fine, isStrafe);
    powVals[ROTATE] = setPowVals(accelValues[ROTATE], rightX_Fine, isRotate);
          
    reportWarning("Forward Power: " + powVals[FORWARD]);
    reportWarning("Strafe Power: " + powVals[STRAFE]);
    reportWarning("Rotate Power: " + powVals[ROTATE]);
    reportWarning("Speed mod: " + STICK_MOD);
    
    frontRight.set(setMechanumSpeed(powVals[FORWARD], powVals[STRAFE], powVals[ROTATE], 0));
    frontLeft.set(setMechanumSpeed(powVals[FORWARD], powVals[STRAFE], powVals[ROTATE], 1));
    rearRight.set(setMechanumSpeed(powVals[FORWARD], powVals[STRAFE], powVals[ROTATE], 2));
    rearLeft.set(setMechanumSpeed(powVals[FORWARD], powVals[STRAFE], powVals[ROTATE], 3));
    
    incSpeed();
    decSpeed();

    stickTotal = 0; // Reset stick total value so it doesn't skyrocket over time
  }

  @Override
  public void teleopPeriodic() {
    if(debugMode == 0) {
      mechDriveCRW();
    } else {
      debugHandle(debugMode);
    }
  }
}
