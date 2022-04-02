package frc.robot.auton;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.ShooterSupersystem;

public class ShootFrontBack extends Auton {

    public ShootFrontBack(DifferentialDrive drive, Intake intake, ShooterSupersystem shooter) {
        super(drive, intake, shooter);
        // TODO Auto-generated constructor stub
    }

    @Override
    public void periodic() {
        switch (state) {
            case 0:
                shooter.getHood().homePeriodic();
                if (shooter.getHood().homed) {
                    nextState();
                }
                break;
            case 1:
                shooter.shootFront();
                if (stateRunTime() > 5000) {
                    shooter.stop();
                    nextState();
                }
                break;
            case 2:
                drive.tankDrive(.7, .7);
                if (stateRunTime() > 4000) {
                    nextState();
                }
                break;
            case 3:
                drive.tankDrive(0, 0);
                break;
        }

    }

}
