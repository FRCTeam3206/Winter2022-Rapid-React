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
  public static final double TURN_TOLERANCE=3;//In Degrees
  public static final double DRIVE_TOLERANCE=4;//In Inches
  public static final double MIN_SPEED=.6;

  //Distance Callabration constants
  public static final double CAL_T1=15.46*Math.PI/180;//Convert to Radians
  public static final double CAL_D=5*12;
  public static final double H1=39.5;
  public static final double H2=8*12+8;
  public static final double T0=Math.atan((H2-H1)/CAL_D)-CAL_T1;
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // m_leftLeadMotor = new CANSparkMax(leftLeadDeviceID, MotorType.kBrushed);
    // m_leftFollowMotor = new CANSparkMax(leftFollowDeviceID, MotorType.kBrushed);
    // m_rightLeadMotor = new CANSparkMax(rightLeadDeviceID, MotorType.kBrushed);
    // m_rightFollowMotor = new CANSparkMax(rightFollowDeviceID, MotorType.kBrushed);

    // m_leftFollowMotor.follow(m_leftLeadMotor);
    // m_rightFollowMotor.follow(m_rightLeadMotor);

    // m_leftLeadMotor.setInverted(true);

    // m_myRobot = new DifferentialDrive(m_leftLeadMotor, m_rightLeadMotor);
    // m_joystick = new XboxController(0);

    // CameraServer.startAutomaticCapture();

  }

  double currRight=0;
  double currLeft=0;
  
  @Override
  public void teleopPeriodic() {
    // double forward=m_joystick.getLeftY()*.5;
    // double turn=-m_joystick.getLeftX()*.7;
    // if(m_joystick.getAButton()){
    //   double[] vals=alignToTarget(72, true, true);
    //   forward=vals[0];
    //   turn=vals[1];
    // }
    // m_myRobot.arcadeDrive(forward,turn);
  }
  @Override
  public void autonomousPeriodic() {
    alignToTarget(80,false,true);
  }
  private double dist(double y){
    y=y*Math.PI/180;
    return (H2-H1)/(Math.tan(y+T0));
  }
  private double cut(double val){
    if(val<-1)return -1;
    if(val>1)return 1;
    return val;
  }
  /**
   * 
   * @param distance desired distance to the target
   * @param shouldTurn if the robot should turn toward the target
   * @param shouldDrive if the robot should drive to or from the target
   * @return  2 doubles. First is the forward speed of the bot, second is it's turn, and the third is if the robot is aligned(1.0 for is, 0.0 for is not)
   */
  private double[] alignToTarget(double distance,boolean shouldTurn,boolean shouldDrive){
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double tv=table.getEntry("tv").getDouble(0.0);//If limelight sees a target
    double tx=table.getEntry("tx").getDouble(0.0);
    int direction=0;
    double turn=0;
    double forward=0;
    double dist=dist(table.getEntry("ty").getDouble(0.0));
    double distOff=dist-72;
    double isOff=0;
    if(tv>.5){
      if(Math.abs(tx)>TURN_TOLERANCE&&shouldTurn){
        turn=-tx/22*.7;
        if(turn>0&&turn<MIN_SPEED)turn=MIN_SPEED;
        if(turn<0&&turn>-MIN_SPEED)turn=-MIN_SPEED;
        //System.out.println(direction+" "+tx+" "+speed);
        isOff=1;
      }
      if(Math.abs(distOff)>DRIVE_TOLERANCE&&shouldDrive){
        if(distOff<0){
          forward=1;
        }else{
          forward=-1;
        }
        if(Math.abs(distOff)<12){
          forward*=.35;
        }else{
          forward*=.5;
        }
        isOff=1;
      }
    }
    //System.out.println(forward+" "+distOff);
    return new double[]{forward,turn,1.0-isOff};
  }
}
