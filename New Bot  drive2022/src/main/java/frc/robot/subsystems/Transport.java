package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.XboxController;



public class Transport extends Subsystem{
    VictorSPX antiJam,kicker;
    XboxController joystick;
    public Transport(int antiJamId,int kickerId,XboxController joystick){
        //antiJam=new VictorSPX(antiJamId);
        kicker=new VictorSPX(kickerId);
        this.joystick=joystick;
    }
    public void periodic(){
        /*if(joystick.getLeftBumper()){
            antiJam.set(VictorSPXControlMode.PercentOutput, .5);
        }else{
            antiJam.set(VictorSPXControlMode.PercentOutput,0);
        }*/
        kicker.set(VictorSPXControlMode.PercentOutput, joystick.getLeftTriggerAxis());
    }
    @Override
    public void init() {
        
    }
}