// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */

public class Robot extends TimedRobot {
  private DifferentialDrive driveTrain;
  private XboxController m_joystick;
  private static final int leftLeadDeviceID = 1; 
  private static final int rightLeadDeviceID = 2;
  private static final int leftFollowDeviceID = 3;
  private static final int rightFollowDeviceID = 4;
  private CANSparkMax m_leftLeadMotor;
  private CANSparkMax m_rightLeadMotor;
  private CANSparkMax m_leftFollowMotor;
  private CANSparkMax m_rightFollowMotor;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftLeadMotor = new CANSparkMax(leftLeadDeviceID, MotorType.kBrushless);
    m_leftFollowMotor = new CANSparkMax(leftFollowDeviceID, MotorType.kBrushless);
    m_rightLeadMotor = new CANSparkMax(rightLeadDeviceID, MotorType.kBrushless);
    m_rightFollowMotor = new CANSparkMax(rightFollowDeviceID, MotorType.kBrushless);

    m_leftFollowMotor.follow(m_leftLeadMotor);
    m_rightFollowMotor.follow(m_rightLeadMotor);

    m_leftLeadMotor.setInverted(true);

    driveTrain = new DifferentialDrive(m_leftLeadMotor, m_rightLeadMotor);
    m_joystick = new XboxController(0);

    CameraServer.startAutomaticCapture();
  }

  double currRight=0;
  double currLeft=0;
  
  @Override
  public void teleopPeriodic() {
     double rightInput=cut(m_joystick.getLeftY()+m_joystick.getLeftX());
     double leftInput=cut(m_joystick.getLeftY()-m_joystick.getLeftX());
     m_leftLeadMotor.set(leftInput);
     m_rightLeadMotor.set(rightInput);
     driveTrain.tankDrive(rightInput, leftInput);
  }
  
  private double cut(double val){
    if(val<-1)return -1;
    if(val>1)return 1;
    return val;
  }
}