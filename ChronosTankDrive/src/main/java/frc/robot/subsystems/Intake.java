package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.XboxController;

public class Intake {
    XboxController joystick;
    VictorSPX intakeMotor;
    public Intake(int motorId,XboxController joystick){
        this.joystick=joystick;
        intakeMotor=new VictorSPX(motorId);
    }
    public void intakePeriodic(){
        intakeMotor.set(VictorSPXControlMode.PercentOutput,joystick.getRightTriggerAxis());
    }
}
