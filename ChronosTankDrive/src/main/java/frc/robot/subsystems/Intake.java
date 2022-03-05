package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Intake extends Subsystem{
    XboxController joystick;
    VictorSPX intakeMotor;
    Solenoid deploy;
    public Intake(int motorId,int pistonId1,XboxController joystick){
        this.joystick=joystick;
        intakeMotor=new VictorSPX(motorId);
        deploy=new Solenoid(PneumaticsModuleType.CTREPCM,pistonId1);
    }
    boolean deployed=false;
    public void periodic(){
        intakeMotor.set(VictorSPXControlMode.PercentOutput, joystick.getLeftX());
        if(joystick.getRightBumperPressed()){
            deploy.toggle();
        }
        
    }
    @Override
    public void init() {
        // TODO Auto-generated method stub
        
    }
}
