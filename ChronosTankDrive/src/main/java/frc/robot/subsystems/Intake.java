package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake {
    XboxController joystick;
    VictorSPX intakeMotor;
    DoubleSolenoid deploy;
    public Intake(int motorId,int pistonId1,int pistonId2,XboxController joystick){
        this.joystick=joystick;
        intakeMotor=new VictorSPX(motorId);
        deploy=new DoubleSolenoid(PneumaticsModuleType.CTREPCM,pistonId1,pistonId2);
    }
    boolean deployed=false;
    public void intakePeriodic(){
        intakeMotor.set(VictorSPXControlMode.PercentOutput,joystick.getRightTriggerAxis());
        if(joystick.getRightBumper()!=deployed){
            deployed=joystick.getRightBumper();
            if(deployed) deploy.set(Value.kForward);
            else deploy.set(Value.kReverse);
        }
    }
}
