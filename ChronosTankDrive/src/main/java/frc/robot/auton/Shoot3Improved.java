package frc.robot.auton;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.subsystems.BallChaser;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.ShooterSupersystem;

public class Shoot3Improved extends Auton {

    BallChaser chaser;

    public Shoot3Improved(DifferentialDrive drive, Intake intake, ShooterSupersystem shooter, BallChaser chaser) {
        super(drive, intake, shooter);
        this.chaser = chaser;
    }

    @Override
    public void periodic() {
        switch (state) {
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
                if (stateRunTime() >= 1500) {

                    nextState();
                }
                break;
            case 2:
                if (stateRunTime() >= 500) {
                    intake.getDeploy().set(false);
                    intake.getMotor().set(VictorSPXControlMode.PercentOutput, 0);
                    nextState();
                }
                break;
            case 3:
                drive.tankDrive(.7, .7);
                if (stateRunTime() >= 500) {
                    nextState();
                }
                break;
            case 4:
                drive.tankDrive(.7, -.7);
                if (stateRunTime() >= 1250) {
                    nextState();
                }
                break;
            case 5:
                shooter.alignAndShoot();
                if (stateRunTime() >= 3000) {
                    shooter.stop();
                    nextState();
                }
                break;
            case 6:
                drive.tankDrive(.7, -.7);
                if (stateRunTime() >= 500) {
                    nextState();
                }
                break;
            case 7:
                if (chaser.chase() || stateRunTime() > 3000) {
                    intake.getDeploy().set(true);
                    intake.getMotor().set(VictorSPXControlMode.PercentOutput, .6);
                    nextState();
                }
                break;
            case 8:
                if (stateRunTime() >= 1500) {
                    intake.getDeploy().set(false);
                    intake.getMotor().set(VictorSPXControlMode.PercentOutput, 0);
                    nextState();
                }
                break;
            case 9:
                drive.tankDrive(.7, -.7);
                if (stateRunTime() >= 1750) {
                    nextState();
                }
                break;
            case 10:
                shooter.alignAndShoot();
                if (stateRunTime() >= 3000) {
                    nextState();
                }
                break;
        }
    }

}