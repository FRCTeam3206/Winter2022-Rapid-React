package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class main extends TimedRobot {
    VictorSPX _Spx0 = new VictorSPX(0);
    VictorSPX _Spx1 = new VictorSPX(1);
    VictorSPX _Spx2 = new VictorSPX(2);
    VictorSPX _Spx3 = new VictorSPX(3);
    Joystick _Joystick = new Joystick(0);

    @Override
    public void teleopPeriodic() {
        double stick = _Joystick.getRawAxis(0);
        _Spx0.set(ControlMode.PercentOutput, stick);
        _Spx1.set(ControlMode.PercentOutput, stick);
        _Spx2.set(ControlMode.PercentOutput, stick);
        _Spx3.set(ControlMode.PercentOutput, stick);

    }
}
