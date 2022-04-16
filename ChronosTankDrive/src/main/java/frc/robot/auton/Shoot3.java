package frc.robot.auton;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.subsystems.BallChaser;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.ShooterSupersystem;

public class Shoot3 extends Auton{

    BallChaser chaser;

    public Shoot3(DifferentialDrive drive, Intake intake, ShooterSupersystem shooter, BallChaser chaser) {
        super(drive, intake, shooter);
        this.chaser = chaser;
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
                if(stateRunTime()>=750){
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
                drive.tankDrive(.7, -.7);
                if(stateRunTime()>=1250){
                    nextState();
                }
                break;
            case 4:
                shooter.alignAndShoot();
                if(stateRunTime()>=3000){
                    shooter.stop();
                    nextState();
                }
				break;
            case 5:
            drive.tankDrive(-.7, .7);
            if(stateRunTime()>=500){
                nextState();
            }
            break;
			case 6:
				if(chaser.chase()||stateRunTime()>3000){
					intake.getDeploy().set(true);
					intake.getMotor().set(VictorSPXControlMode.PercentOutput, .6);
					nextState();
				}
				break;
			case 7:
                drive.tankDrive(.7, .7);
                if(stateRunTime()>=1000){
                    nextState();
                }
                break;
			case 8:
                drive.tankDrive(.5, -.5);
                if(stateRunTime()>=1000){
                    nextState();
                }
                break;
			case 9:
                shooter.alignAndShoot();
                if(stateRunTime()>=3000){
                    nextState();
                }
				break;
        }
    }
    
}