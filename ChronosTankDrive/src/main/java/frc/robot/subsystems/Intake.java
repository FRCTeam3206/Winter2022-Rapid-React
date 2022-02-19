package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake extends Subsystem{
    XboxController joystick;
    VictorSPX intakeMotor;
    Solenoid rightDeploy,leftDeploy;
    public Intake(int motorId,int pistonId1,int pistonId2,XboxController joystick){
        this.joystick=joystick;
        intakeMotor=new VictorSPX(motorId);
        rightDeploy=new Solenoid(PneumaticsModuleType.CTREPCM,pistonId1);
        leftDeploy=new Solenoid(PneumaticsModuleType.CTREPCM,pistonId2);
    }
    boolean deployed=false;
    public void periodic(){
        intakeMotor.set(VictorSPXControlMode.PercentOutput,joystick.getRightTriggerAxis());
        if(joystick.getRightBumperPressed()){
            rightDeploy.toggle();
            leftDeploy.toggle();
        }
    }
    @Override
    public void init() {
        // TODO Auto-generated method stub
        
    }
}
