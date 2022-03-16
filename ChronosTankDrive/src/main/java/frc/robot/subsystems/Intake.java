package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import static frc.robot.Constants.Buttons.*;

public class Intake extends Subsystem{
    GenericHID joystick;
    VictorSPX intakeMotor;
    Solenoid deploy;
    boolean on = false;

    public Intake(int motorId,int pistonId1,GenericHID joystick){
        this.joystick=joystick;
        intakeMotor=new VictorSPX(motorId);
        deploy=new Solenoid(PneumaticsModuleType.CTREPCM,pistonId1);
        deploy.set(on);
    }
    boolean deployed=false;
    public void periodic(){
        if(joystick.getRawButton(B_INTAKE)){
            intakeMotor.set(ControlMode.PercentOutput, 1);
        }else if(joystick.getRawButton(B_REVERSE_INTAKE)){
            intakeMotor.set(ControlMode.PercentOutput, -1);
        }else{
            intakeMotor.set(ControlMode.PercentOutput, 0);
        }
        if(joystick.getRawButtonPressed(B_DEPLOY)){
            deploy.toggle();
        }
        
    }
    @Override
    public void init() {
        // TODO Auto-generated method stub
        
    }
}
