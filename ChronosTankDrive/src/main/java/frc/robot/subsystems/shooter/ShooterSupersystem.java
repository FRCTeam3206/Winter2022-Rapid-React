package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Subsystem;
import static frc.robot.Constants.Buttons.*;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class ShooterSupersystem extends Subsystem {
    private Shooter shooter;
    private Hood hood;
    private Limelight limelight;
    private DifferentialDrive driveTrain;
    private GenericHID joystick1, joystick2;
    private boolean aligned = false;
    private VictorSPX agitator;

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
        this.agitator = new VictorSPX(Constants.IDS.AGITATE_PORT);
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
                * 180 / Math.PI * Constants.Shooter.HOOD_MULTIPLIER;
    }

    public void agitate() {
        if (System.currentTimeMillis() / 500 % 3 < 1)// It works, albeit questionably
            agitator.set(VictorSPXControlMode.PercentOutput, .42);
        else
            stopAgitate();
    }

    public void stopAgitate() {
        agitator.set(VictorSPXControlMode.PercentOutput, 0);
    }

    public boolean alignTo(double angle, double distance) {
        agitate();
        double turn = -angle / 20;
        SmartDashboard.putNumber("Turn", turn);
        if (Math.abs(turn) < .03) {
            turn = 0;
        }
        driveTrain.arcadeDrive(0, turn);
        double hoodAngle = hoodAngle(distance);
        SmartDashboard.putNumber("Hood Angle", hoodAngle);
        hood.setAngle(hoodAngle);
        shooter.setSpeed(-getRPMFromDistance(distance));
        return turn < .2 && Math.abs(hood.getAngle() - hoodAngle) < 2;
    }

    public boolean align() {
        double[] distanceAndAngle = limelight.getAdjustedDistanceAndAngleToTarget();
        double distance = distanceAndAngle[0];
        double angle = distanceAndAngle[1];
        return alignTo(angle, distance);
    }

    public void shoot(double distance) {// 4.045*distance+2374.7 is from pure testing
        // New 5.57x+2014
        agitate();
        shooter.shoot(getRPMFromDistance(distance));
        // shooter.shoot(SmartDashboard.getNumber("RPM",0.0));
    }

    public double getRPMFromDistance(double distance) {
        return 5.57 * (distance + 18) + 2014;
    }

    public void shoot() {
        double[] distanceAndAngle = limelight.getAdjustedDistanceAndAngleToTarget();
        double distance = distanceAndAngle[0];
        shoot(distance);
    }

    public void shootFront() {
        hood.setAngle(10);
        if (Math.abs(hood.getAngle() - 10) < 1) {
            shooter.shoot(2575);
        }
        agitate();
    }

    public void shootLow() {
        hood.setAngle(25);
        if (Math.abs(hood.getAngle() - 25) < 1) {
            shooter.shoot(2000);
        }
        agitate();
    }

    public void stop() {
        shooter.stop();
        stopAgitate();
    }

    public void alignAndShoot() {
        double[] distanceAndAngle = limelight.getAdjustedDistanceAndAngleToTarget();
        double distance = distanceAndAngle[0];
        SmartDashboard.putNumber("Calculated RPM", 4.045 * distance + 2374.7);
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
        if (joystick1.getRawButton(7)) {
            shoot();
        }
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
                stop();
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
        if (joystick2.getRawButton(1)) {
            shootLow();
        }
        SmartDashboard.putNumber("Hood Angle", hood.angle);
    }
}

// when bumper is against the hub to shoot, hood angle should be 78 degrees, and
// rpm should be 2300
