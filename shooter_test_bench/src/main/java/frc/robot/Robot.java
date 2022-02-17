// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.motors.Hood;
import frc.robot.motors.Shooter;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  

  private Shooter shooter;
  private Hood hood;

  private XboxController m_joystick;

  private static final int kShooterPort = 1;
  private static final int kJoystickPort = 0;
  private static final double increment = 0.01;

  private static final int kHoodPort = 2;

  @Override
  public void robotInit() {
    m_joystick = new XboxController(kJoystickPort);
    shooter = new Shooter(kShooterPort, increment, m_joystick);
    hood = new Hood(kHoodPort, m_joystick);
  }

  

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    shooter.showEncoderValOnSmartDashboard();
  }

  @Override
  public void teleopPeriodic() {
    shooter.shooterPeriodic();
    hood.hoodPeriodic();

		// double targetPositionRotations = m_joystick.getLeftY() * 10.0 * 4096;
		// m_hoodMotor.set(ControlMode.Position, targetPositionRotations);
  }

  
}
