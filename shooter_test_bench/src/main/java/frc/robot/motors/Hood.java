package frc.robot.motors;

import javax.swing.text.html.HTML.Tag;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Hood {
    private WPI_TalonSRX m_hoodMotor;
    private XboxController m_joystick;

    private double pos;
    private double inc = 0.05;

    public Hood(int port, XboxController joystick) {
        m_joystick = joystick;

        m_hoodMotor = new WPI_TalonSRX(port);

        m_hoodMotor.configFactoryDefault();

        /* Config the sensor used for Primary PID and sensor direction */
        m_hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);

        /* Ensure sensor is positive when output is positive */
        m_hoodMotor.setSensorPhase(Constants.kSensorPhase);

        /**
         * Set based on what direction you want forward/positive to be.
         * This does not affect sensor phase.
         */
        m_hoodMotor.setInverted(Constants.kMotorInvert);

        /* Config the peak and nominal outputs, 12V means full */
        m_hoodMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
        m_hoodMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
        m_hoodMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
        m_hoodMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        /**
         * Config the allowable closed-loop error, Closed-Loop output will be
         * neutral within this range. See Table in Section 17.2.1 for native
         * units per rotation.
         */
        m_hoodMotor.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

        /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
        m_hoodMotor.config_kF(Constants.kPIDLoopIdx, Constants.hood_kF, Constants.kTimeoutMs);
        m_hoodMotor.config_kP(Constants.kPIDLoopIdx, Constants.hood_kP, Constants.kTimeoutMs);
        m_hoodMotor.config_kI(Constants.kPIDLoopIdx, Constants.hood_kI, Constants.kTimeoutMs);
        m_hoodMotor.config_kD(Constants.kPIDLoopIdx, Constants.hood_kD, Constants.kTimeoutMs);

        /**
         * Grab the 360 degree position of the MagEncoder's absolute
         * position, and intitally set the relative sensor to match.
         */
        int absolutePosition = m_hoodMotor.getSensorCollection().getPulseWidthPosition();

        /* Mask out overflows, keep bottom 12 bits */
        absolutePosition &= 0xFFF;
        if (Constants.kSensorPhase) {
            absolutePosition *= -1;
        }
        if (Constants.kMotorInvert) {
            absolutePosition *= -1;
        }

        /* Set the quadrature (relative) sensor to match absolute */
        m_hoodMotor.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }

    public void hoodPeriodic() {

        if (m_joystick.getPOV() == 90) {
            pos = 0.0;
          } else if (m_joystick.getPOV() == 0) {
            pos += inc;
          } else if (m_joystick.getPOV() == 180) {
            pos -= inc;
          } else if (m_joystick.getPOV() == 270) {
              pos = 0.0;
              m_hoodMotor.setSelectedSensorPosition(0);
          }
        double targetPositionRotations =  pos * 10.0 * 4096;
		    //System.out.println(targetPositionRotations);
        m_hoodMotor.set(ControlMode.Position, -targetPositionRotations);
        SmartDashboard.putNumber("Hood Position", targetPositionRotations);
    }
}
