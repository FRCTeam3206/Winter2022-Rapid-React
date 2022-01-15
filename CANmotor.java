/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick; // this should be just the sticks on the logitech controller
  private Joystick m_rightStick;
  private static final int leftDeviceID = 1; 
  private static final int leadDeviceID = 1;
  private static final int followDeviceID = 2;
  private static final int rightDeviceID = 3;
  private static final int leadDeviceID2 = 3; //called ID2 for the motor on the right side
  private static final int followDeviceID2 = 4;
  
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;

  private CANSparkMax m_leadMotor;
  private CANSparkMax m_followMotor;

  @Override
  public void robotInit() {
   
    m_leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushless); // these do say brushless motors which shouldn't be a problem.
    m_leadMotor = new CANSparkMax(leadDeviceID, MotorType.kBrushless);
    m_followMotor = new CANSparkMax(followDeviceID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushless);
    m_leadMotor = new CANSparkMax(leadDeviceID2, MotorType.kBrushless);
    m_followMotor = new CANSparkMax(followDeviceID2, MotorType.kBrushless);

    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);

    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());
  }
}
