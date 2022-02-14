// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  private static final int kShooterPort = 1;
  private static final int kHoodPort = 2;
  private static final int kJoystickPort = 0;

  private static final double increment = 0.01;

  private XboxController m_joystick;

  private CANSparkMax m_shooter;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private double shooterSetPoint = 0;

  private WPI_TalonSRX m_hood;
  public double kP_Hood, kI_Hood, kD_Hood, kFF_Hood;


  @Override
  public void robotInit() {
    shooterInit();
    hoodInit();
  }

  private void shooterInit() {
    m_shooter = new CANSparkMax(kShooterPort, MotorType.kBrushless);
    m_joystick = new XboxController(kJoystickPort);
    // m_encoder = new Encoder(kEncoderPortA, kEncoderPortB);

    m_shooter.restoreFactoryDefaults();

    m_pidController = m_shooter.getPIDController();

    m_encoder = m_shooter.getEncoder();

    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.
    // m_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    kP = Constants.sparkmax_kP;
    kI = Constants.sparkmax_kI;
    kD = Constants.sparkmax_kD; 
    kIz = Constants.sparkmax_kIz; 
    kFF = Constants.sparkmax_kFF; 
    kMaxOutput = Constants.sparkmax_kMaxOut; 
    kMinOutput = Constants.sparkmax_kMinOut;
    maxRPM = 5700;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Shooter P Gain", kP);
    SmartDashboard.putNumber("Shooter I Gain", kI);
    SmartDashboard.putNumber("Shooter D Gain", kD);
    SmartDashboard.putNumber("Shooter I Zone", kIz);
    SmartDashboard.putNumber("Shooter Feed Forward", kFF);
    SmartDashboard.putNumber("Shooter Max Output", kMaxOutput);
    SmartDashboard.putNumber("Shooter Min Output", kMinOutput);

  }

  private void hoodInit() {
    m_hood = new WPI_TalonSRX(kHoodPort);

    //========================= Set up hood-angle controlling motor =========================//
    m_hood.configFactoryDefault();
		
		/* Config the sensor used for Primary PID and sensor direction */
    m_hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		/* Ensure sensor is positive when output is positive */
		m_hood.setSensorPhase(Constants.kSensorPhase);

		/**
		 * Set based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. 
		 */ 
		m_hood.setInverted(Constants.kMotorInvert);

		/* Config the peak and nominal outputs, 12V means full */
		m_hood.configNominalOutputForward(0, Constants.kTimeoutMs);
		m_hood.configNominalOutputReverse(0, Constants.kTimeoutMs);
		m_hood.configPeakOutputForward(1, Constants.kTimeoutMs);
		m_hood.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		m_hood.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		m_hood.config_kF(Constants.kPIDLoopIdx, Constants.hood_kF, Constants.kTimeoutMs);
		m_hood.config_kP(Constants.kPIDLoopIdx, Constants.hood_kP, Constants.kTimeoutMs);
		m_hood.config_kI(Constants.kPIDLoopIdx, Constants.hood_kI, Constants.kTimeoutMs);
		m_hood.config_kD(Constants.kPIDLoopIdx, Constants.hood_kD, Constants.kTimeoutMs);

		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		int absolutePosition = m_hood.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		absolutePosition &= 0xFFF;
		if (Constants.kSensorPhase) { absolutePosition *= -1; }
		if (Constants.kMotorInvert) { absolutePosition *= -1; }
		
		/* Set the quadrature (relative) sensor to match absolute */
		m_hood.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    
  }


  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Encoder", m_encoder.getVelocity());
  }

  @Override
  public void teleopPeriodic() {
    shooterPeriodic();
    // hoodPeriodic();
    
		// double targetPositionRotations = m_joystick.getLeftY() * 10.0 * 4096;
		// m_hoodMotor.set(ControlMode.Position, targetPositionRotations);
  }

  private void shooterPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    
    if (m_joystick.getBButtonPressed()) {
      shooterSetPoint = 0.0;
    } else if (m_joystick.getYButtonPressed()) {
      shooterSetPoint += increment;
    } else if (m_joystick.getAButtonPressed()) {
      shooterSetPoint -= increment;
    }
    shooterSetPoint = MathUtil.clamp(shooterSetPoint, kMinOutput, kMaxOutput);

    double rpm_target = shooterSetPoint*maxRPM;

    m_pidController.setReference(rpm_target, CANSparkMax.ControlType.kVelocity);
    
    SmartDashboard.putNumber("SetPoint", shooterSetPoint);
    SmartDashboard.putNumber("RPM_Target", rpm_target);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
    
		// double targetPositionRotations = m_joystick.getLeftY() * 10.0 * 4096;
		// m_hoodMotor.set(ControlMode.Position, targetPositionRotations);
  }
}
