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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private XboxController m_joystick;
  private static final int leftLeadDeviceID = 1; 
  private static final int rightLeadDeviceID = 3;
  private static final int leftFollowDeviceID = 2;
  private static final int rightFollowDeviceID = 4;
  private CANSparkMax m_leftLeadMotor;
  private CANSparkMax m_rightLeadMotor;
  private CANSparkMax m_leftFollowMotor;
  private CANSparkMax m_rightFollowMotor;
  private double lastVelocity=0;
  private long lastTime;

  private NetworkTableEntry distance;
  private double MIN_SPEED=0.45;
  private double TURN_TOLERANCE=2;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftLeadMotor = new CANSparkMax(leftLeadDeviceID, MotorType.kBrushed);
    m_leftFollowMotor = new CANSparkMax(leftFollowDeviceID, MotorType.kBrushed);
    m_rightLeadMotor = new CANSparkMax(rightLeadDeviceID, MotorType.kBrushed);
    m_rightFollowMotor = new CANSparkMax(rightFollowDeviceID, MotorType.kBrushed);

    m_leftFollowMotor.follow(m_leftLeadMotor);
    m_rightFollowMotor.follow(m_rightLeadMotor);

    m_leftLeadMotor.setInverted(true);

    m_myRobot = new DifferentialDrive(m_leftLeadMotor, m_rightLeadMotor);
    m_joystick = new XboxController(0);

    lastTime=System.currentTimeMillis();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    distance = inst.getTable("BallLocator").getEntry("Distance from center");
  }

  double currRight=0;
  double currLeft=0;
  
  @Override
  public void autonomousPeriodic() {
    //for acceleration limiting(uncomplete)
    //  double rightInput=cut(m_joystick.getLeftY()+m_joystick.getLeftX());
    //  double leftInput=cut(m_joystick.getLeftY()-m_joystick.getLeftX());
    //  long currTime=System.currentTimeMillis();
    //  double rightDiff=rightInput-currRight;
    //  double leftDiff=leftInput-currLeft;
    //  double timeDiff=currTime-lastTime;

    double d = distance.getDouble(0);
    System.out.println(d);
    double rightin = 0, leftin = 0;
    if (Math.abs(d) > TURN_TOLERANCE) {
      rightin = (d - (-80)) / (80 + 80) * (-1 - 1) + 1;
      leftin = (d - (-80)) / (80 + 80) * (-1 - 1) + 1;

      if (rightin < MIN_SPEED) rightin = MIN_SPEED;
      if (leftin < MIN_SPEED) leftin = MIN_SPEED;
    }
    m_myRobot.tankDrive(leftin/1.1, -rightin/1.1);
  }

  

  @Override
  public void teleopPeriodic() {
    double rightin = m_joystick.getRawAxis(1);
    double leftin = m_joystick.getRawAxis(3);
     
    m_myRobot.tankDrive(rightin/1.35, leftin/1.35);
  }
  
  private double cut(double val){
    if(val<-1)return -1;
    if(val>1)return 1;
    return val;
  }
}
