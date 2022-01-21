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
  private long lastTime;

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

    CameraServer.startAutomaticCapture();

    lastTime=System.currentTimeMillis();
  }

  double currRight=0;
  double currLeft=0;
  int prints=0;
  @Override
  public void teleopPeriodic() {
    //for acceleration limiting(uncomplete)
    double rightInput=cut(m_joystick.getLeftY()+m_joystick.getLeftX());
    double leftInput=cut(m_joystick.getLeftY()-m_joystick.getLeftX());
    long currTime=System.currentTimeMillis();
    long elapsedTime=currTime-lastTime;
    double rightDiff=rightInput-currRight;
    double leftDiff=leftInput-currLeft;
    double maxDiv=1;
    double maxAccel=.25;
    //rightDiff*1000/elapsedTime is essentially the slope of the line connecting the last rightMotor speed and the one given by the input(with respect to seconds)
    if(Math.abs(rightDiff*1000/elapsedTime)>.1){
    maxDiv=Math.max(maxDiv,Math.abs(rightDiff*1000/elapsedTime/maxAccel));
    }
    if(Math.abs(leftDiff*1000/elapsedTime)>.1){
    maxDiv=Math.max(maxDiv,Math.abs(leftDiff*1000/elapsedTime/maxAccel));
    }
    
    if(elapsedTime<25){
      if(prints<25){
        System.out.println(elapsedTime+" "+rightDiff+" "+maxDiv+" "+currRight+" "+currRight+rightDiff/maxDiv+" "+rightInput);
        prints++;
      }
      currRight=cut(currRight+rightDiff/maxDiv);
      currLeft=cut(currLeft+leftDiff/maxDiv);
      //stop when the joysticks are released
      if(Math.abs(m_joystick.getLeftY())<.05&&Math.abs(m_joystick.getLeftX())<.05){
        currRight=0;
        currLeft=0;
      }
    }
    m_myRobot.tankDrive(currRight, currLeft);
    lastTime=System.currentTimeMillis();
  }
  
  private double cut(double val){
    if(val<-1)return -1;
    if(val>1)return 1;
    return val;
  }
}
