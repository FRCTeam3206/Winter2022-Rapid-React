package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;

public class Hood extends Subsystem {
  private WPI_TalonSRX m_hoodMotor;
  private GenericHID m_joystick;

  private double angle = ZERO_POS;
  private double inc = .25;
  private static final double ZERO_POS = 27.5;
  public double kP, kI, kD, kIz, kFF;

  public Hood(int port, GenericHID joystick) {
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
  public void init(){}
  public void periodic() {
    if (m_joystick.getPOV() == 90) {
      angle = ZERO_POS;
    } else if (m_joystick.getPOV() == 0) {
      angle += inc;
    } else if (m_joystick.getPOV() == 180) {
      angle -= inc;
    } else if (m_joystick.getPOV() == 270) {
      angle = ZERO_POS;
      m_hoodMotor.setSelectedSensorPosition(0);
    }

    // System.out.println(pos);
    // setPos(60.0);
    double targetPositionRotations = (angle - ZERO_POS) / 3.07 * 40960;
    m_hoodMotor.set(ControlMode.Position, targetPositionRotations);

    this.updateSmartDashboard();
    SmartDashboard.putNumber("Target Position Rotations", targetPositionRotations);
    SmartDashboard.putNumber("Current Position", m_hoodMotor.getSelectedSensorPosition());
  }

  public double getAngle() {
    return angle;
  }
}
