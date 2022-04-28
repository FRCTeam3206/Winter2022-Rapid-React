package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;

import java.lang.Math;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.auton.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSupersystem;

import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.IDS.*;
import static frc.robot.Constants.Limelight.*;
import static frc.robot.Constants.Drive.*;
import static frc.robot.Constants.Pneumatics.*;

import edu.wpi.first.wpilibj.AnalogInput;
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
  Joystick rightStick;
  Joystick leftStick;
  XboxController weaponStick;

  // Sendable Chooser
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
  double tLateDrive = 18;// longest time it takes to drive for any function
  double tLateTurn = 15;// longest time it takes to turn for any function

  Timer velocityTimer = new Timer();

  // DriveTrain Pneumatics
  Solenoid driveSol = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  // PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  AnalogInput pressureInput = new AnalogInput(0);
  Limelight limelight;
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

  Subsystem[] subSystems;
  ShooterSupersystem shooter;
  Intake intake;
  GyroControl gyroControl;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightStick = new Joystick(0);
    leftStick = new Joystick(2);
    weaponStick = new XboxController(1);
    // For later notes: assign buttons for when we are on red/blue alliance so that
    // we can
    // identify what color balls we have in transport and whether to shoot them or
    // to get rid of them

    leftFrontDrive = new CANSparkMax(1, MotorType.kBrushless);
    rightFrontDrive = new CANSparkMax(2, MotorType.kBrushless);
    leftBackDrive = new CANSparkMax(3, MotorType.kBrushless);
    rightBackDrive = new CANSparkMax(4, MotorType.kBrushless);

    leftEncoder = leftFrontDrive.getEncoder();
    rightEncoder = rightFrontDrive.getEncoder();

    leftFrontDrive.restoreFactoryDefaults();
    rightFrontDrive.restoreFactoryDefaults();
    leftBackDrive.restoreFactoryDefaults();
    rightBackDrive.restoreFactoryDefaults();
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    autoChoices.setDefaultOption("Shoot2", "Shoot2");
    autoChoices.addOption("Shoot2", "Shoot2");
    autoChoices.addOption("ShootFrontNoBack", "ShootFrontNoBack");
    autoChoices.addOption("ShootFrontBack", "ShootFrontBack");
    autoChoices.addOption("Shoot3", "Shoot3");
    SmartDashboard.putData(autoChoices);
    rightFrontDrive.setInverted(true);
    leftBackDrive.follow(leftFrontDrive);
    rightBackDrive.follow(rightFrontDrive);
    limelight = new Limelight(24, 32 + 3, 9.5, 16.5);
    chronosDrive = new DifferentialDrive(leftFrontDrive, rightFrontDrive);
    intake = new Intake(INTAKE_MOTOR_PORT, INTAKE_DEPLOY_PORT, weaponStick);
    shooter = new ShooterSupersystem(new Shooter(SHOOT_PORT, KICKER_PORT, .01, leftStick),
        new Hood(HOOD_PORT, HOOD_LIMIT_PORT, leftStick), limelight, chronosDrive, leftStick, weaponStick);
    subSystems = new Subsystem[] { intake, shooter, new Climber(8, 10, 7, 6, weaponStick) };
    for (Subsystem subSystem : subSystems) {
      subSystem.init();
    }
    gyroControl = new GyroControl(chronosDrive);
    CameraServer.startAutomaticCapture();
  }

  public void accelLimit(double leftInput, double rightInput) {
    rightAdjusted = (1 / accelDriveKonstant) * rightInput
        + (accelDriveKonstant - 1) / accelDriveKonstant * rightAdjusted;
    leftAdjusted = (1 / accelDriveKonstant) * leftInput + (accelDriveKonstant - 1) / accelDriveKonstant * leftAdjusted;
    // chronosDrive.tankDrive(leftAdjusted, rightAdjusted);
  }

  public void teleopInit() {

  }

  /*
   * public void autonomousInit(){
   * shooter.getHood().init();
   * }
   */
  @Override
  public void teleopPeriodic() {
    gyroControl.getAngle();
    if (rightStick.getRawButton(5)) {

    } else {
      if (!leftStick.getRawButton(Constants.Buttons.B_ALIGN)) {
        if (accelerationLimiting) {
          accelLimit(leftStick.getY(), rightStick.getY());
        } else {
          rightAdjusted = rightStick.getY();
          leftAdjusted = leftStick.getY();
        }

        if (TURN_LIMIT) {
          // this would be simpler if we switched to arcadeDrive()
          double forwardLimit = (rightAdjusted + leftAdjusted) / 2.0;
          double turnLimit = ((leftAdjusted - rightAdjusted) / 2.0);
          turnLimit = Math.pow(turnLimit, 2) * Math.signum(turnLimit);

          leftAdjusted = forwardLimit + turnLimit;
          rightAdjusted = forwardLimit - turnLimit;
        }

        if (rightStick.getRawButton(1)) {
          driveSol.set(true);
        } else {
          driveSol.set(false);
        }

        chronosDrive.tankDrive(leftAdjusted, rightAdjusted);

      }
    }
    // make toggle button to switch between automated shooting and manual shooting
    // and have the value for
    // shooterSpeed only be taken into account when on manual

    // Trigger is shoot on right

    // intake/extake button assignments
    for (Subsystem subSystem : subSystems) {
      subSystem.periodic();
    }

    if (COMPRESSOR_TOGGLE_ENABLED && weaponStick.getRawButtonReleased(9)) {
      if (compressor.enabled()) {
        compressor.disable();
      } else {
        compressor.enableDigital();
      }
    }

    SmartDashboard.putNumber("Current Max Pressure", 250.0 * (pressureInput.getVoltage() / 5.0) - 25.0);
  }

  public void autoAlignShoot() {
    while (!shooter.align()) {
      shooter.getHood().update();
    }
    long start = System.currentTimeMillis();
    while (start + 5000 > System.currentTimeMillis())
      shooter.shoot();
    shooter.stop();
  }

  private Auton selectedAutoRoutine;

  @Override
  public void autonomousInit() {
    autoSelected = autoChoices.getSelected();
    switch (autoSelected) {
      case "Shoot1":
        selectedAutoRoutine = new Shoot1(chronosDrive, intake, shooter);
        break;
      case "Shoot2":
        selectedAutoRoutine = new Shoot2(chronosDrive, intake, shooter);
        break;
      case "ShootFrontBack":
        selectedAutoRoutine = new ShootFrontBack(chronosDrive, intake, shooter);
        break;
      case "ShootFrontNoBack":
        selectedAutoRoutine = new ShootFrontNoBack(chronosDrive, intake, shooter);
        break;
      case "Shoot3":
        selectedAutoRoutine = new Shoot3V3(chronosDrive, intake, shooter, gyroControl);
        break;
    }
  }

  public void delay(long time) {
    long start = System.currentTimeMillis();
    while (start + time >= System.currentTimeMillis())
      ;
  }

  public void autonomousPeriodic() {
    selectedAutoRoutine.periodic();
  }
}