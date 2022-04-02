package frc.robot.auton;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.ShooterSupersystem;

public class Shoot2 extends Auton{

    public Shoot2(DifferentialDrive drive, Intake intake, ShooterSupersystem shooter) {
        super(drive, intake, shooter);
    }

    @Override
    public void periodic() {
        switch(state){
            case 0:
                shooter.getHood().homePeriodic();
                intake.getDeploy().set(true);
                intake.getMotor().set(VictorSPXControlMode.PercentOutput, .6);
                if (shooter.getHood().homed) {
                    nextState();
                }
                break;
            case 1:
                drive.tankDrive(-.7, -.7);
                if(stateRunTime()>=3000){
                    intake.getMotor().set(VictorSPXControlMode.PercentOutput, 0);
                    intake.getDeploy().set(false);
                    nextState();
                }
                break;
            case 2:
                drive.tankDrive(.7, .7);
                if(stateRunTime()>=500){
                    nextState();
                }
                break;
            case 3:
                drive.tankDrive(.5, -.5);
                if(stateRunTime()>=2750){
                    nextState();
                }
                break;
            case 4:
                shooter.alignAndShoot();
                if(stateRunTime()>=4000){
                    nextState();
                }
            
        }
    }
    
}
