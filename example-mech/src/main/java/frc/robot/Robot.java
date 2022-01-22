// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  @Override
  public void robotInit() {
    // 8 controllers for 8 motors
    VictorSP frontLeft = new VictorSP(kFrontLeftChannel);    // Front Left
    VictorSP rearLeft = new VictorSP(kRearLeftChannel);       // Back Left
    VictorSP frontRight = new VictorSP(kFrontRightChannel);  // Front Right
    VictorSP rearRight = new VictorSP(kRearRightChannel);     // Back Right

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    //frontRight.setInverted(true);
    //rearRight.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_stick = new XboxController(kXBOXChannel);
  }

  @Override
  public void teleopPeriodic() {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
    m_robotDrive.driveCartesian(m_stick.getLeftX(), m_stick.getLeftY(), m_stick.getRightX(), 0.0);
      // The left stick isn't working for some reason. Don't know if it's code or mechanical
  }
}
