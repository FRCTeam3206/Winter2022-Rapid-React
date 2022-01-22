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
  
  @Override
  public void teleopPeriodic() {
    
  }
  @Override
  public void autonomousPeriodic() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double ta=table.getEntry("ta").getDouble(0.0);//If limelight sees a target
    double tx=table.getEntry("tx").getDouble(0.0);
    int direction=0;
    if(ta>.75){
        if(tx>5){
          direction=1;
        }else if(tx<5){
          direction=-1;
        }
    }
    System.out.println(direction);

    switch(direction){
      case 1:
        m_myRobot.arcadeDrive(0, .4);
        break;
      case -1:
        m_myRobot.arcadeDrive(0,-.4);
        break;
      default:
        m_myRobot.arcadeDrive(0, 0);
        break;
    }

  }
  private double cut(double val){
    if(val<-1)return -1;
    if(val>1)return 1;
    return val;
  }
}