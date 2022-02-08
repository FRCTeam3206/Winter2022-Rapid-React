package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    VictorSPX _Spx0 = new VictorSPX(0);
    VictorSPX _Spx1 = new VictorSPX(1);
    VictorSPX _Spx2 = new VictorSPX(2);
    VictorSPX _Spx3 = new VictorSPX(3);
    Joystick _Joystick = new Joystick(0);

    @Override
    public void teleopPeriodic() {
        double stickX = _Joystick.getRawAxis(0);
        double stickY = _Joystick.getRawAxis(1);

        _Spx2.follow(_Spx1);
        _Spx3.follow(_Spx0);

        stickY = stickY * -1;
        
        if (stickX > 0.10) {
            // Turn Right
            _Spx0.set(ControlMode.PercentOutput, stickX);
            _Spx1.set(ControlMode.PercentOutput, stickX);
        } else if (stickX < -0.10) {
            // Turn Left
            _Spx0.set(ControlMode.PercentOutput, stickX);
            _Spx1.set(ControlMode.PercentOutput, stickX);
        } else if (stickY > 0.10) {
            // Move Forward
            _Spx0.set(ControlMode.PercentOutput, -stickY);
            _Spx1.set(ControlMode.PercentOutput, stickY);
        } else if (stickY < -0.10) {
            // Move Backwards
            _Spx0.set(ControlMode.PercentOutput, stickY);
            _Spx1.set(ControlMode.PercentOutput, -stickY);
        } else {
            // Stop
            _Spx0.set(ControlMode.PercentOutput, 0);
            _Spx1.set(ControlMode.PercentOutput, 0);
        }

    }
}
