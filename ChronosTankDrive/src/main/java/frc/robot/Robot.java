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

//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.cscore.UsbCamera;
//import com.analog.adis16448.frc.ADIS16448_IMU;

//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.AddressableLED;
//import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs
 * the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  // Drive Type
  // Joysticks
  Joystick leftStick;
  Joystick rightStick;
  XboxController weaponStick;

  // Sendable Chooser
  // SendableChooser<String> autoChoices = new SendableChooser<>();
  String autoSelected;

  // the folliwing four lines are part of the original basic code
  /*
   * private final PWMSparkMax leftFrontDrive = new PWMSparkMax(0);
   * private final PWMSparkMax rightFrontDrive = new PWMSparkMax(1);
   * private final DifferentialDrive m_robotDrive = new
   * DifferentialDrive(leftFrontDrive, rightFrontDrive);
   * private final Joystick m_stick = new Joystick(0);
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
  DifferentialDrive chronosDrive;
  // Drivetrain Motors
  CANSparkMax rightFrontDrive;
  CANSparkMax leftFrontDrive;
  CANSparkMax leftBackDrive;
  CANSparkMax rightBackDrive;

  String DesiredDirection;
  double desiredDistance;
  double distanceTraveled;
  double tLateDrive = 18;// longest time it takes to drive for any function
  double tLateTurn = 15;// longest time it takes to turn for any function

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
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // chronosDrive = new DifferentialDrive(leftFrontDrive, rightFrontDrive);
    leftStick = new Joystick(0);
    rightStick = new Joystick(1);
    weaponStick=new XboxController(0);
    // For later notes: assign buttons for when we are on red/blue alliance so that
    // we can
    // identify what color balls we have in transport and whether to shoot them or
    // to get rid of them

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
    /*
     * autoChoices.setDefaultOption("AutoLine", "AutoLine");
     * autoChoices.addOption("BallPickup", "BallPickup");
     * autoChoices.addOption("Shoot", "Shoot");
     */
    rightBackDrive.setInverted(true);
    leftBackDrive.follow(leftFrontDrive);
    rightBackDrive.follow(rightFrontDrive);
    /*
     * //this allows the driver to manually move transport forward and backward
     * backBallTransport.configSelectedFeedbackSensor(FeedbackDevice.
     * CTRE_MagEncoder_Relative, 0, 30);
     * backBallTransport.configNominalOutputForward(0, 30);
     * backBallTransport.configNominalOutputReverse(0, 30);
     * backBallTransport.configPeakOutputForward(1, 30);
     * backBallTransport.configPeakOutputReverse(-1, 30);
     */
    chronosDrive = new DifferentialDrive(leftFrontDrive, rightFrontDrive);
  }

  @Override
  public void teleopPeriodic() {
    /*
     * //joystick drive
     * if (rightStick.getRawButton(1)) { // Low Speed
     * driveSol.set(Value.kForward);
     * } else if (rightStick.getRawButton(2)) { // High Speed
     * driveSol.set(Value.kReverse);
     * } else {
     * driveSol.set(Value.kOff); // Ensures Pistons are Off
     * 
     * }
     */
    if(accelerationLimiting){
      rightAdjusted=(1/accelDriveKonstant)*leftStick.getY()+(accelDriveKonstant-1)/accelDriveKonstant*rightAdjusted;
      leftAdjusted=(1/accelDriveKonstant)*rightStick.getY()+(accelDriveKonstant-1)/accelDriveKonstant*leftAdjusted;
    }else{
      rightAdjusted=rightStick.getY();
      leftAdjusted=leftStick.getY();
    }
    chronosDrive.tankDrive(rightAdjusted,leftAdjusted);

    // make toggle button to switch between automated shooting and manual shooting
    // and have the value for
    // shooterSpeed only be taken into account when on manual

    // Trigger is shoot on right

    // intake/extake button assignments
  }

  public void Drive(double distance) {
    distanceTraveled = leftEncoder.getPosition() * -1 / 12;
    desiredDistance = distance + distanceTraveled;
    velocityTimer.start();
    /*
     * DriveLabel: if (distance > 0) {
     * while (desiredDistance > distanceTraveled) {
     * distanceTraveled = leftEncoder.getPosition() * -1 / 12;
     * ballToggleButton = ballSwitch.get();
     * chronosDrive.tankDrive(-.6, -.6);
     * // Set Button to Integer Value
     * if (ballToggleButton == false && ballToggle == 0) { // First Press
     * ballToggle = 1; // If trigger is pressed and toggle hasn't been set yet/has
     * cycled through then
     * // toggle = 1
     * } else if (ballToggleButton == true && ballToggle == 1) { // First Release
     * ballToggle = 2; // If trigger is released and toggle = 1 then toggle = 2
     * }
     * // Determine Piston Position Based on Integer Value
     * if (ballToggle == 1 || ballToggle == 2) { // Trigger is Pressed
     * if (frontBallTransport.getSelectedSensorPosition() * ballMagScale <
     * ballDesiredDistance) {
     * frontBallTransport.set(ControlMode.PercentOutput, -.7);
     * backBallTransport.set(ControlMode.PercentOutput, -.7);
     * } else if (frontBallTransport.getSelectedSensorPosition() * ballMagScale >
     * ballDesiredDistance) {
     * ballToggle = 0;
     * frontBallTransport.stopMotor();
     * backBallTransport.stopMotor();
     * frontBallTransport.setSelectedSensorPosition(0);
     * }
     * }
     */
    chronosDrive.tankDrive(-7, 7);
    /*
     * if (velocityTimer.get() >= tLateDrive) {
     * break DriveLabel;
     * }
     * }
     * } else if (distance < 0) {
     * while (desiredDistance < distanceTraveled) {
     * distanceTraveled = leftEncoder.getPosition() * -1 / 12;
     * ballToggleButton = ballSwitch.get();
     * // Set Button to Integer Value
     * if (ballToggleButton == false && ballToggle == 0) { // First Press
     * ballToggle = 1; // If trigger is pressed and toggle hasn't been set yet/has
     * cycled through then
     * // toggle = 1
     * } else if (ballToggleButton == true && ballToggle == 1) { // First Release
     * ballToggle = 2; // If trigger is released and toggle = 1 then toggle = 2
     * }
     * // Determine Piston Position Based on Integer Value
     * if (ballToggle == 1 || ballToggle == 2) { // Trigger is Pressed
     * if (frontBallTransport.getSelectedSensorPosition() * ballMagScale <
     * ballDesiredDistance) {
     * frontBallTransport.set(ControlMode.PercentOutput, -.7);
     * backBallTransport.set(ControlMode.PercentOutput, -.7);
     * } else if (frontBallTransport.getSelectedSensorPosition() * ballMagScale >
     * ballDesiredDistance) {
     * ballToggle = 0;
     * frontBallTransport.stopMotor();
     * backBallTransport.stopMotor();
     * frontBallTransport.setSelectedSensorPosition(0);
     * }
     * }
     * chronosDrive.tankDrive(.8, .8);
     * if (velocityTimer.get() >= tLateDrive) {
     * break DriveLabel;
     * }
     * }
     * }
     */
  }

  @Override
  public void autonomousInit() {
    // autoSelected = autoChoices.getSelected();

    compressor.enableDigital();
    driveSol.set(Value.kReverse);
    /*
     * switch (autoSelected) {
     * case "AutoLine":
     * Drive(9); //will want to drive at least the length of the robot forward, must
     * be fully off tarmac to get easy points
     * break;
     * case "BallPickup":
     * BallPickup();
     * break;
     * case "Shoot":
     * Shoot();
     * break;
     * 
     * }
     */
    while (Timer.getMatchTime() < 8) {
      chronosDrive.tankDrive(.5, -.5); // left should be negative, right should be positive
    }
  }

  public void autonomousPeriodic() {
  }
}