package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;

public class Hood extends Subsystem {
  private WPI_TalonSRX m_hoodMotor;
  private GenericHID m_joystick;

  private double angle = Constants.Shooter.HOOD_ZERO_POS;
  private double inc = .25;
  public double kP, kI, kD, kIz, kFF;
  private DigitalInput limit;

  public Hood(int port, int limitPort, GenericHID joystick) {
    m_joystick = joystick;

    m_hoodMotor = new WPI_TalonSRX(port);

    m_hoodMotor.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    m_hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.Shooter.kPIDLoopIdx,
        Constants.Shooter.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    m_hoodMotor.setSensorPhase(Constants.Shooter.kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be.
     * This does not affect sensor phase.
     */
    m_hoodMotor.setInverted(Constants.Shooter.kMotorInvert);

    /* Config the peak and nominal outputs, 12V means full */
    m_hoodMotor.configNominalOutputForward(0, Constants.Shooter.kTimeoutMs);
    m_hoodMotor.configNominalOutputReverse(0, Constants.Shooter.kTimeoutMs);
    m_hoodMotor.configPeakOutputForward(0.25, Constants.Shooter.kTimeoutMs);
    m_hoodMotor.configPeakOutputReverse(-0.25, Constants.Shooter.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be
     * neutral within this range. See Table in Section 17.2.1 for native
     * units per rotation.
     */
    m_hoodMotor.configAllowableClosedloopError(0, Constants.Shooter.kPIDLoopIdx, Constants.Shooter.kTimeoutMs);

    kP = Constants.Shooter.hood_kP;
    kI = Constants.Shooter.hood_kI;
    kD = Constants.Shooter.hood_kD;
    kFF = Constants.Shooter.sparkmax_kFF;

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    m_hoodMotor.config_kF(Constants.Shooter.kPIDLoopIdx, kFF, Constants.Shooter.kTimeoutMs);
    m_hoodMotor.config_kP(Constants.Shooter.kPIDLoopIdx, kP, Constants.Shooter.kTimeoutMs);
    m_hoodMotor.config_kI(Constants.Shooter.kPIDLoopIdx, kI, Constants.Shooter.kTimeoutMs);
    m_hoodMotor.config_kD(Constants.Shooter.kPIDLoopIdx, kD, Constants.Shooter.kTimeoutMs);

    /* Set the quadrature (relative) sensor to match absolute */
    m_hoodMotor.setSelectedSensorPosition(0, Constants.Shooter.kPIDLoopIdx, Constants.Shooter.kTimeoutMs);

    // limit=new DigitalInput(limitPort);
  }

  public void updateSmartDashboard() {
    double p = SmartDashboard.getNumber("P Gain (hood)", Constants.Shooter.hood_kP);
    double i = SmartDashboard.getNumber("I Gain (hood)", Constants.Shooter.hood_kI);
    double d = SmartDashboard.getNumber("D Gain (hood)", Constants.Shooter.hood_kD);
    double ff = SmartDashboard.getNumber("Feed Forward (hood)", Constants.Shooter.hood_kF);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kP)) {
      m_hoodMotor.config_kP(Constants.Shooter.kPIDLoopIdx, p, Constants.Shooter.kTimeoutMs);
      kP = p;
    }
    if ((i != kI)) {
      m_hoodMotor.config_kI(Constants.Shooter.kPIDLoopIdx, i, Constants.Shooter.kTimeoutMs);
      kI = i;
    }
    if ((d != kD)) {
      m_hoodMotor.config_kD(Constants.Shooter.kPIDLoopIdx, d, Constants.Shooter.kTimeoutMs);
      kD = d;
    }
    if ((ff != kFF)) {
      m_hoodMotor.config_kF(Constants.Shooter.kPIDLoopIdx, ff, Constants.Shooter.kTimeoutMs);
      kFF = ff;
    }

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Feed Forward (hood)", kFF);
    SmartDashboard.putNumber("P Gain (hood)", kP);
    SmartDashboard.putNumber("I Gain (hood)", kI);
    SmartDashboard.putNumber("D Gain (hood)", kD);
  }

  public void init() {
    home();
  }

  public void setHome() {
    angle = Constants.Shooter.HOOD_ZERO_POS;
    m_hoodMotor.setSelectedSensorPosition(0);
  }

  public void home() {
    /*
     * m_hoodMotor.set(.5);
     * while(!limit.get());
     * m_hoodMotor.set(-.25);
     * while(limit.get());
     * m_hoodMotor.set(0);
     * setHome();
     */
  }

  public void periodic() {
    if (m_joystick.getPOV() == 90) {
      angle = Constants.Shooter.HOOD_ZERO_POS;
    } else if (m_joystick.getPOV() == 0) {
      angle += inc;
    } else if (m_joystick.getPOV() == 180) {
      angle -= inc;
    } else if (m_joystick.getPOV() == 270) {
      angle = Constants.Shooter.HOOD_ZERO_POS;
      m_hoodMotor.setSelectedSensorPosition(0);
    }

    // System.out.println(pos);
    // setPos(60.0);

    update();

  }

  public void update() {
    double targetPositionRotations = (angle - Constants.Shooter.HOOD_ZERO_POS) * Constants.Shooter.HOOD_TEETH_ANGLE_RAT
        / Constants.Shooter.HOOD_SPOOL_TEETH * Constants.Shooter.HOOD_TICKS_PER_ROTATION;
    m_hoodMotor.set(ControlMode.Position, targetPositionRotations);
    this.updateSmartDashboard();
    SmartDashboard.putNumber("Target Position Rotations", targetPositionRotations);
    SmartDashboard.putNumber("Current Position", m_hoodMotor.getSelectedSensorPosition());
  }

  public void setAngle(double angle) {
    angle = MathUtil.clamp(angle, Constants.Shooter.HOOD_ZERO_POS,
        Constants.Shooter.HOOD_ZERO_POS + Constants.Shooter.HOOD_RANGE);
    this.angle = angle;
  }

  public double getAngle() {
    return m_hoodMotor.getSelectedSensorPosition() / Constants.Shooter.HOOD_TICKS_PER_ROTATION
        * Constants.Shooter.HOOD_SPOOL_TEETH / Constants.Shooter.HOOD_TEETH_ANGLE_RAT + Constants.Shooter.HOOD_ZERO_POS;
  }

}
