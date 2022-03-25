package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Climber extends Subsystem {
    VictorSPX motor1, motor2;
    GenericHID joystick;

    public Climber(int port1, int port2, GenericHID joystick) {
        this.motor1 = new VictorSPX(port1);
        this.motor2 = new VictorSPX(port2);
        motor2.follow(motor1);
        this.joystick = joystick;
    }

    @Override
    public void init() {
        // TODO Auto-generated method stub

    }

    @Override
    public void periodic() {
        if (joystick.getPOV() == 0) {
            motor1.set(ControlMode.PercentOutput, 1);
        } else if (joystick.getPOV() == 180) {
            motor1.set(ControlMode.PercentOutput, -1);
        } else {
            motor1.set(ControlMode.PercentOutput, 0);
        }

    }

}
