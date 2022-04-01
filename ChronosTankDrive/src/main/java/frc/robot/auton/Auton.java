package frc.robot.auton;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.ShooterSupersystem;

public abstract class Auton {
    protected long stateStart;
    public DifferentialDrive drive;
    public Intake intake;
    public ShooterSupersystem shooter;
    public int state = 0;

    public Auton(DifferentialDrive drive, Intake intake, ShooterSupersystem shooter) {
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        stateStart = System.currentTimeMillis();
    }

    public abstract void periodic();

    public void nextState() {
        stateStart = System.currentTimeMillis();
        state++;
    }

    public long stateRunTime() {
        return System.currentTimeMillis() - stateStart;
    }
}
