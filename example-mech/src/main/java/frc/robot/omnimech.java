// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  private double accelValues[];
  private static int FORWARD = 0;
  private static int STRAFE = 1;
  private static int ROTATE = 2;

  private static int ACCEL_CO = 8; // Accelleration coefficient
  private static double STICK_MOD = 0.5; // Artifical limit on input range
  private static double STICK_DEADZONE = 0.1; // xbox controller deadband

  private double stickTotal = 0;  // Total values of all stick inputs outside the deadband

  private int fakeBool_Forward = 0;
  private int fakeBool_Strafe = 0;
  private int fakeBool_Rotate = 0;

  private double leftX_Fine = 0;
  private double leftY_Fine = 0;
  private double rightX_Fine = 0;

  private double powVals[];

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

  @Override
  public void teleopPeriodic() {

    fakeBool_Forward = (m_stick.getLeftY() >= STICK_DEADZONE || m_stick.getLeftY() <= -STICK_DEADZONE) ? 1 : 0;   // Set the fake boolean to "true" or "false" on whether the stick is outside of the deadband
    fakeBool_Strafe = (m_stick.getLeftX() >= STICK_DEADZONE || m_stick.getLeftX() <= -STICK_DEADZONE) ? 1 : 0;    // --
    fakeBool_Rotate = (m_stick.getRightX() >= STICK_DEADZONE || m_stick.getRightX() <= -STICK_DEADZONE) ? 1 : 0;  // --

    if(fakeBool_Forward == 1) {
      stickTotal += realValue(m_stick.getLeftY());
    }
    if(fakeBool_Strafe == 1) {
      stickTotal += realValue(m_stick.getLeftX());
    }
    if(fakeBool_Rotate == 1) {
      stickTotal += realValue(m_stick.getRightX());
    }

    leftY_Fine = m_stick.getLeftY() * (realValue(m_stick.getLeftY()) / stickTotal) * fakeBool_Forward;
    leftX_Fine = m_stick.getLeftX() * (realValue(m_stick.getLeftX()) / stickTotal) * fakeBool_Strafe;
    rightX_Fine = m_stick.getRightX() * (realValue(m_stick.getRightX()) / stickTotal) * fakeBool_Rotate;

    // Accelleration formula so the motor speed isn't crazy right off the bat
    accelValues[FORWARD] = (1 / ACCEL_CO) * (m_stick.getLeftY() * -STICK_MOD) + ((ACCEL_CO - 1) / ACCEL_CO);
    accelValues[STRAFE] = (1 / ACCEL_CO) * (m_stick.getLeftX() * STICK_MOD) + ((ACCEL_CO - 1) / ACCEL_CO);
    accelValues[ROTATE] = (1 / ACCEL_CO) * (m_stick.getRightX() * STICK_MOD) + ((ACCEL_CO - 1) / ACCEL_CO);

    powVals[FORWARD] = accelValues[FORWARD] * leftY_Fine;
    powVals[STRAFE] = accelValues[STRAFE] * leftX_Fine;
    powVals[ROTATE] = accelValues[ROTATE] * rightX_Fine;

    // Idea:
      // Use weighted averages to set the controller's fine tuning
        // Find whether a stick axis is greater / less than the deadzone
        // If so, add its value to the total & flip a boolean
        // Divide each stick's value by the total, then multiply the value with the previous result
        // Multiply the stick values by the weight value

    frontRight.set(powVals[FORWARD] - powVals[STRAFE] - powVals[ROTATE]);
    frontLeft.set(powVals[FORWARD] + powVals[STRAFE] + powVals[ROTATE]);
    rearRight.set(powVals[FORWARD] + powVals[STRAFE] - powVals[ROTATE]);
    rearLeft.set(powVals[FORWARD] - powVals[STRAFE] + powVals[ROTATE]);

    stickTotal = 0; // Reset stick total value so it doesn't skyrocket over time
  }
}
