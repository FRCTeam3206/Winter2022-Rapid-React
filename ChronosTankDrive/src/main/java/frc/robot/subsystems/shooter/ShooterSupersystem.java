package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Subsystem;
import static frc.robot.Constants.Buttons.*;

public class ShooterSupersystem extends Subsystem {
    private Shooter shooter;
    private Hood hood;
    private Limelight limelight;
    private DifferentialDrive driveTrain;
    private GenericHID joystick1, joystick2;
    private boolean aligned = false;

    public Hood getHood() {
        return hood;
    }

    public ShooterSupersystem(Shooter shooter, Hood hood, Limelight limelight, DifferentialDrive driveTrain,
            GenericHID joystick1, GenericHID joystick2) {
        this.shooter = shooter;
        this.hood = hood;
        this.limelight = limelight;
        this.driveTrain = driveTrain;
        this.joystick1 = joystick1;
        this.joystick2 = joystick2;
    }

    @Override
    public void init() {
        shooter.init();
        aligned = false;
        // hood.init();
        SmartDashboard.putNumber("RPM", 0);
    }

    public double hoodAngle(double distance) {
        return 90 - Math.atan((Constants.Shooter.SHOOTER_HEIGHT_DIFF
                + Math.sqrt(Math.pow(Constants.Shooter.SHOOTER_HEIGHT_DIFF, 2) + Math.pow(distance, 2))) / distance)
                * 180 / Math.PI;
    }

    public boolean alignTo(double angle, double distance) {
        double turn = -angle / 25;
        SmartDashboard.putNumber("Turn", turn);
        if (Math.abs(turn) < .03) {
            turn = 0;
        }
        driveTrain.arcadeDrive(0, turn);
        double hoodAngle = hoodAngle(distance);
        SmartDashboard.putNumber("Hood Angle", hoodAngle);
        hood.setAngle(hoodAngle);
        return turn < .2 && Math.abs(hood.getAngle() - hoodAngle) < 2;
    }

    public boolean align() {
        double[] distanceAndAngle = limelight.getAdjustedDistanceAndAngleToTarget();
        double distance = distanceAndAngle[0];
        double angle = distanceAndAngle[1];
        return alignTo(angle, distance);
    }

    public void shoot(double distance) {// 4.045*distance+2374.7 is from pure testing
        shooter.shoot(4.045 * distance + 2374.7 + 0);
        // shooter.shoot(SmartDashboard.getNumber("RPM",0.0));
    }

    public void shoot() {
        double[] distanceAndAngle = limelight.getAdjustedDistanceAndAngleToTarget();
        double distance = distanceAndAngle[0];
        shoot(distance);
    }

    public void shootFront() {
        hood.setAngle(10);
        if (Math.abs(hood.getAngle() - 10) < 1) {
            shooter.shoot(2500);
        }
    }

    public void stop() {
        shooter.stop();
    }

    public void alignAndShoot() {
        double[] distanceAndAngle = limelight.getAdjustedDistanceAndAngleToTarget();
        double distance = distanceAndAngle[0];
        SmartDashboard.putNumber("Distance", distance);
        double angle = distanceAndAngle[1];
        aligned = align();
        if (aligned) {
            shoot(distance);
        }
    }

    @Override
    public void periodic() {
        // boolean shooting=false; // I don't think this is used
        double[] distanceAndAngle;
        double distance;
        double angle;
        double distAway;
        // boolean aligned = false;
        double turn;
        double forward;
        if (joystick2.getRawButton(B_SHOOTER_FAILSAFE)) {
            shootFront();
        } else {

            // aligned = false;
            turn = 0;
            forward = 0;
            if (joystick1.getRawButton(B_ALIGN) && limelight.sees()) {
                // aligned=alignTo(angle, distance);
                alignAndShoot();
            } else {
                shooter.stop();
                aligned = false;
            }
            // hood.setAngle(20);//There will be a function based on ll to find this
        }
        SmartDashboard.putBoolean("Aligned", aligned);
        shooter.showEncoderValOnSmartDashboard();
        if (joystick2.getRawButtonPressed(B_HOME))
            hood.resetHomed();
        if (joystick2.getRawButton(B_HOME))
            hood.homePeriodic();
        if (joystick2.getRawButton(8)) {
            shooter.invert();
        }
    }
}

// when bumper is against the hub to shoot, hood angle should be 78 degrees, and
// rpm should be 2300
