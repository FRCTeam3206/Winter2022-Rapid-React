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

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cscore.UsbCamera;
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
    //boolean XboxDrive = true;
    // Joysticks
    Joystick leftStick;
    Joystick rightStick;
    //XboxController driveStick;  
    //XboxController weaponStick;

  // Sendable Chooser
  SendableChooser<String> autoChoices = new SendableChooser<>();
  String autoSelected;

  //the folliwing four lines are part of the original basic code
 /*
  private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final Joystick m_stick = new Joystick(0);
*/
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
  DifferentialDrive morpheusDrive;
  // Drivetrain Motors
  CANSparkMax leftFrontDrive;
  CANSparkMax rightFrontDrive;
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

  //Intake 
  //values,motors,and motor controller types are subject to change
  WPI_VictorSPX intakeMotor = new WPI_VictorSPX(5);
  DoubleSolenoid intakeSol = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 3, 4);
  Timer intakeTimer = new Timer();
  // Intake Toggle Variables
  int intakeToggle = 0;
  boolean intakeToggleStick;
  boolean extakeToggleStick;
  int extakeToggle = 0;
  // Motor Speed
  double intakeSpeed = .65;

  //transport
  WPI_TalonSRX frontBallTransport = new WPI_TalonSRX(10);
  WPI_TalonSRX backBallTransport = new WPI_TalonSRX(11);
  DigitalInput ballSwitch = new DigitalInput(3);
  double ballCount = 0; //this is the initial # of balls in transport at the biginning of a match
  double ballMagScale = .00110390625;// (1/4096 * 3.14 * 1.972) scaled to inches instead
  double ballDesiredDistance = 5.5;
  double ballDistanceTraveled;
  int ballToggle = 0;
  boolean ballToggleButton;


  //shooter 
  int shooterToggle = 0;
  boolean shooterToggleStick;
  boolean shooter2ToggleStick;
  int shooter2Toggle = 0;
  double hoodAngle; //manual shooting
  double shooterSpeed; //manual shooting
  boolean shootingStyle; //automated shooting (limelight) vs. manual shooting (slider value)


  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    morpheusDrive = new DifferentialDrive(leftFrontDrive, rightFrontDrive);
      leftStick = new Joystick(0);
      rightStick = new Joystick(1);

    // For later notes: assign buttons for when we are on red/blue alliance so that we can 
    //identify what color balls we have in transport and whether to shoot them or to get rid of them

    leftFrontDrive = new CANSparkMax(1, MotorType.kBrushless);
    rightFrontDrive = new CANSparkMax(3, MotorType.kBrushless);
    leftBackDrive = new CANSparkMax(2, MotorType.kBrushless);
    rightBackDrive = new CANSparkMax(4, MotorType.kBrushless);
    leftEncoder = leftFrontDrive.getEncoder();
    rightEncoder = rightFrontDrive.getEncoder();

    leftFrontDrive.restoreFactoryDefaults();
    rightFrontDrive.restoreFactoryDefaults();
    leftBackDrive.restoreFactoryDefaults();
    rightBackDrive.restoreFactoryDefaults();
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    autoChoices.setDefaultOption("AutoLine", "AutoLine");
    autoChoices.addOption("BallPickup", "BallPickup");
    autoChoices.addOption("Shoot", "Shoot");

    leftBackDrive.follow(leftFrontDrive);
    rightBackDrive.follow(rightFrontDrive);

    //this allows the driver to manually move transport forward and backward
    backBallTransport.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    backBallTransport.configNominalOutputForward(0, 30);
    backBallTransport.configNominalOutputReverse(0, 30);
    backBallTransport.configPeakOutputForward(1, 30);
    backBallTransport.configPeakOutputReverse(-1, 30);

    morpheusDrive = new DifferentialDrive(leftFrontDrive, rightFrontDrive);
   
    //m_rightMotor is part of original code
   // m_rightMotor.setInverted(true);
    
  }

  @Override
  public void teleopPeriodic() {
    //joystick drive
    if (rightStick.getRawButton(1)) { // Low Speed
      driveSol.set(Value.kForward);
    } else if (rightStick.getRawButton(2)) { // High Speed
      driveSol.set(Value.kReverse);
    } else {
      driveSol.set(Value.kOff); // Ensures Pistons are Off
    }
    morpheusDrive.tankDrive(leftStick.getY(), rightStick.getY());
    
    //make toggle button to switch between automated shooting and manual shooting and have the value for 
    //shooterSpeed only be taken into account when on manual

    //Trigger is shoot on right

    //intake/extake button assignments
  }
  public void Drive(double distance) {
    distanceTraveled = leftEncoder.getPosition() * -1 / 12;
    desiredDistance = distance + distanceTraveled;
    velocityTimer.start();
    DriveLabel: if (distance > 0) {
      while (desiredDistance > distanceTraveled) {
        distanceTraveled = leftEncoder.getPosition() * -1 / 12;
        ballToggleButton = ballSwitch.get();
        morpheusDrive.tankDrive(-.6, -.6);
     // Set Button to Integer Value
     if (ballToggleButton == false && ballToggle == 0) { // First Press
      ballToggle = 1; // If trigger is pressed and toggle hasn't been set yet/has cycled through then
                      // toggle = 1

    } else if (ballToggleButton == true && ballToggle == 1) { // First Release
      ballToggle = 2; // If trigger is released and toggle = 1 then toggle = 2
    }
    // Determine Piston Position Based on Integer Value
    if (ballToggle == 1 || ballToggle == 2) { // Trigger is Pressed
      if (frontBallTransport.getSelectedSensorPosition() * ballMagScale < ballDesiredDistance) {
        frontBallTransport.set(ControlMode.PercentOutput, -.7);
        backBallTransport.set(ControlMode.PercentOutput, -.7);
      } else if (frontBallTransport.getSelectedSensorPosition() * ballMagScale > ballDesiredDistance) {
        ballToggle = 0;
        frontBallTransport.stopMotor();
        backBallTransport.stopMotor();
        frontBallTransport.setSelectedSensorPosition(0);
      }
    }
    morpheusDrive.tankDrive(-.7, -.7);
    if (velocityTimer.get() >= tLateDrive) {
      break DriveLabel;
    }
  }
} else if (distance < 0) {
  while (desiredDistance < distanceTraveled) {
    distanceTraveled = leftEncoder.getPosition() * -1 / 12;
    ballToggleButton = ballSwitch.get();
    // Set Button to Integer Value
    if (ballToggleButton == false && ballToggle == 0) { // First Press
      ballToggle = 1; // If trigger is pressed and toggle hasn't been set yet/has cycled through then
                      // toggle = 1

    } else if (ballToggleButton == true && ballToggle == 1) { // First Release
      ballToggle = 2; // If trigger is released and toggle = 1 then toggle = 2
    }
    // Determine Piston Position Based on Integer Value
    if (ballToggle == 1 || ballToggle == 2) { // Trigger is Pressed
      if (frontBallTransport.getSelectedSensorPosition() * ballMagScale < ballDesiredDistance) {
        frontBallTransport.set(ControlMode.PercentOutput, -.7);
        backBallTransport.set(ControlMode.PercentOutput, -.7);
      } else if (frontBallTransport.getSelectedSensorPosition() * ballMagScale > ballDesiredDistance) {
        ballToggle = 0;
        frontBallTransport.stopMotor();
        backBallTransport.stopMotor();
        frontBallTransport.setSelectedSensorPosition(0);
      }
    }
    morpheusDrive.tankDrive(.8, .8);
    if (velocityTimer.get() >= tLateDrive) {
      break DriveLabel;
    }
  }
}
}
//have a default shooting setting to fall back on in case of limelight issues



//the following are autonomous only commands
public void BallPickup(){
 //will need to put out intake (requires pneumatic stuffs)
 //run motors to pull balls in
 //run transport to hold balls in bot
}

public void Shoot(){
 // THIS IS THE PRIORITY FOR AUTO
 //move balls up to shooter
 //turn on shooter motors
 //find goal with limelight
 //shoot based on location/distance and stuff
 //know how many balls are in transport so it can know whether or not to keep going
 
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
    case "BallPickup":
      BallPickup();
      break;
    case "Shoot":
      Shoot();
      break;   
    }
  }
  public void autonomousPeriodic(){

  }
  }


