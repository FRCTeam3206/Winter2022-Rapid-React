// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.lang.Math;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import edu.wpi.first.wpilibj.util.Color;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.I2C;
//import com.analog.adis16448.frc.ADIS16448_IMU;

//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.AddressableLED;
//import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
    // Drive Type
    // Joysticks
    Joystick leftStick;
    Joystick rightStick;
    XboxController weaponStick;

  //Sendable Chooser
  SendableChooser<String> autoChoices = new SendableChooser<>();
  String autoSelected;

 // Acceleration Limiting Variables
 boolean accelerationLimiting = true;
 double accelLimitedLeftGetY;
 double accelLimitedRightGetY;
 double accelLimitedSlideDrive;
 double accelDriveKonstant = 6; // Change from 2-32. 32 is super slow to react, 2 is little improvement
 double leftDriveCoef = .7;
 double rightDriveCoef = .7;
 double rightStickDeadband = .1;
 double leftStickDeadband = .1;
 double leftAdjusted;
 double rightAdjusted;

  // DriveTrain
  DifferentialDrive chronosDrive;
  // Drivetrain Motors
  CANSparkMax rightFrontDrive;
  CANSparkMax leftFrontDrive;
  CANSparkMax leftBackDrive;
  CANSparkMax rightBackDrive;

  String DesiredDirection;
  double desiredDistance;
  double distanceTraveled;
  double tLateDrive = 18;//longest time it takes to drive for any function
  double tLateTurn = 15;//longest time it takes to turn for any function

  Timer velocityTimer = new Timer();

  // DriveTrain Pneumatics
  DoubleSolenoid driveSol = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);
  PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  // DriveTrain Encoders
  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;

  // Drive Variables
  double leftDriveSpeed;
  double rightDriveSpeed;
  double triggerDeadzone = .1;
  int desiredDirection;
  double powerNumber = 3;
  double primeConstant = .8; // This needs to be between 0 and 1. 0 means that the drive train is just x^3
                             // and 1 means that it is just x. 1 is more sensitive and 0 is less sensitive.
  double rightPrime;
  double leftPrime;

  double tickTock = Timer.getFPGATimestamp();


  //ball counter for intake transport/shooter
  DigitalInput ballSwitch = new DigitalInput(3);
  double ballCount = 1; //this is the initial # of balls in transport at the beginning of a match
  double ballMagScale = .00110390625;// (1/4096 * 3.14 * 1.972) scaled to inches instead
  double ballDesiredDistance = 5.5;
  double ballDistanceTraveled;
  int ballToggle = 0;
  boolean ballToggleButton;

  DriverStation.Alliance allianceColor = DriverStation.getAlliance();
  Boolean isRedAlliance;

 //Color Sensor 
 I2C.Port i2cPort = I2C.Port.kOnboard;
 ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
 ColorMatch m_colorMatcher = new ColorMatch();
 // Colors
 Color kBlueTarget = new Color(0.2, 0.45, 0.34); //new color values
 Color kRedTarget = new Color(0.4, 0.4, 0.2); //new color values
 //Color kBlueTarget = new Color(0.143, 0.427, 0.429); //old color values
 //Color kRedTarget = new Color(0.561, 0.232, 0.114); //old color values

 String ballColor = "";
 //String ball2Color = "";

 //Subsystem[] subSystems;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //chronosDrive = new DifferentialDrive(leftFrontDrive, rightFrontDrive);
      leftStick = new Joystick(0);
      rightStick = new Joystick(1);
      weaponStick = new XboxController(0);
  
    leftFrontDrive = new CANSparkMax(1, MotorType.kBrushless);
    rightFrontDrive = new CANSparkMax(2, MotorType.kBrushless);
    leftBackDrive = new CANSparkMax(3, MotorType.kBrushless);
    rightBackDrive = new CANSparkMax(4, MotorType.kBrushless);

    chronosDrive = new DifferentialDrive(leftFrontDrive, rightFrontDrive);
    leftEncoder = leftFrontDrive.getEncoder();
    rightEncoder = rightFrontDrive.getEncoder();
  
    leftFrontDrive.restoreFactoryDefaults();
    rightFrontDrive.restoreFactoryDefaults();
    leftBackDrive.restoreFactoryDefaults();
    rightBackDrive.restoreFactoryDefaults();
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    autoChoices.setDefaultOption("AutoLine", "AutoLine");
    autoChoices.addOption("humanPlayerBall", "humanPlayerBall");
    autoChoices.addOption("leftBall", "leftBall");
    autoChoices.addOption("rightBall", "rightBall");
    autoChoices.addOption("centerBall", "centerBall");

    rightFrontDrive.setInverted(true);
    leftBackDrive.follow(leftFrontDrive);
    rightBackDrive.follow(rightFrontDrive);
    chronosDrive = new DifferentialDrive(leftFrontDrive, rightFrontDrive);
    /*
    subSystems = new Subsystem[]{//new intake(5, 0, 1, weaponStick), new Transport(0, 8, weaponStick)
      for(Subsystem subSystem :subSystems){
        subSystem.init();
      }
    }*/

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
  }

  public void accelLimit(double rightInput, double leftInput){
    rightAdjusted=(1/accelDriveKonstant)*leftInput+(accelDriveKonstant-1)/accelDriveKonstant*rightAdjusted;
    leftAdjusted=(1/accelDriveKonstant)*rightInput+(accelDriveKonstant-1)/accelDriveKonstant*leftAdjusted;
  }

  @Override
  public void teleopPeriodic() {
  
    //joystick drive
    if (rightStick.getRawButton(0)) { // Low Speed
      driveSol.set(Value.kForward);
    } else if (rightStick.getRawButton(1)) { // High Speed
      driveSol.set(Value.kReverse);
    } else {
      driveSol.set(Value.kOff); // Ensures Pistons are Off
    }
    chronosDrive.tankDrive(leftStick.getY(), rightStick.getY());

    //color sensor
    Color detectedColor = m_colorSensor.getColor();
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if(match.color == kBlueTarget){
      colorString = "Blue";
    } else if(match.color == kRedTarget){
      colorString = "Red";
    } else{
      colorString = "Unknown";
    }
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putString("Ball 1 Color", ballColor);
    //SmartDashboard.putString("Ball 2 Color", ball2Color);

    if(accelerationLimiting){
      accelLimit(rightStick.getY(), leftStick.getY());
    }else{
      rightAdjusted = rightStick.getY();
      leftAdjusted = leftStick.getY();
    }
  }
/*
  private boolean alignToTarget(double desiredDistance,boolean shouldDrive){
    double[] out=new double[2];
    //This is more computationally efficient, and conserves bandwidth
    if(shouldDrive){
      out=limelight.getDistanceAndAngleToTarget();
    }else{
      out[1]=limelight.getDistanceFromTarget();
    }
    double forward=0;
    boolean adjusted=true;
    if(shouldDrive){
      double distAway=out[0]-desiredDistance;
      if(Math.abs(distAway)>12){
        if(Math.abs(distAway)>36){
          forward=.7;
        }else{
          forward=.5;
        }
        adjusted=false;
      }
      if(distAway<0)forward*=-1;
    }
    double turn=0;
    double angleAway=out[1];
    if(Math.abs(angleAway)>2){
      if(Math.abs(angleAway)>20){
        turn=.7;
      }else{
        turn=.5;
      }
      adjusted=false;
    }
    if(angleAway<0)turn*=-1;
    double rightInput=forward-turn;
    double leftInput=forward+turn;
    accelLimit(rightInput, leftInput);
    return adjusted;
  }
*/
  public void Drive(double distance) {
   distanceTraveled = leftEncoder.getPosition() * -1 / 12;
   desiredDistance = distance + distanceTraveled;
    velocityTimer.start();
    DriveLabel: if (distance > 0) {
      while (desiredDistance > distanceTraveled) {
        distanceTraveled = leftEncoder.getPosition() * -1 / 12;
        chronosDrive.tankDrive(-.6, .6);
        if (velocityTimer.get() >= tLateDrive) {
          break DriveLabel;
        }
      }
    } else if (distance < 0) {
      while (desiredDistance < distanceTraveled) {
        distanceTraveled = leftEncoder.getPosition() * -1 / 12;
        chronosDrive.tankDrive(-.7, .7);
        if (velocityTimer.get() >= tLateDrive) {
          break DriveLabel;
        }
      }
    } else {
      chronosDrive.tankDrive(0, 0);
    }
    chronosDrive.tankDrive(0, 0);
}




//the following are autonomous only commands
public void humanPlayerBall(){
 //will need to put out intake (requires pneumatic stuffs)
 //run motors to pull balls in
 //run transport to hold balls in bot
}

public void leftBall(){
 // THIS IS THE PRIORITY FOR AUTO
 //move balls up to shooter
 //turn on shooter motors
 //find goal with limelight
 //shoot based on location/distance and stuff
 //know how many balls are in transport so it can know whether or not to keep going
 
}

public void rightBall(){

}

public void centerBall(){

}

  @Override
  public void autonomousInit() {
    autoSelected = autoChoices.getSelected();

    compressor.enableDigital();
    driveSol.set(Value.kReverse);
    
    switch (autoSelected) {
    case "AutoLine":
      Drive(9); //will want to drive at least the length of the robot forward, must be fully off tarmac to get easy points
      break;
    case "humanPlayerBall":
      humanPlayerBall();
      break;
    case "leftBall":
      leftBall();
      break;   
    case "rightBall":
      rightBall();
      break;
    case "centerBall":
      centerBall();
      break;
    }
  }
  public void autonomousPeriodic(){
    while (ballCount > 0){
      
    }

    //color sensor
    Color detectedColor = m_colorSensor.getColor();
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if(match.color == kBlueTarget){
      colorString = "Blue";
    } else if(match.color == kRedTarget){
      colorString = "Red";
    } else{
      colorString = "Unknown";
    }
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }
}