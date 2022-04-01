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
                * 180 / Math.PI;
    }

    public void agitate() {
        agitator.set(VictorSPXControlMode.PercentOutput, .5);
    }
    public void stopAgitate(){
        agitator.set(VictorSPXControlMode.PercentOutput, 0);
    }
    public boolean alignTo(double angle, double distance) {
        agitate();
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
        agitate();
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
        stopAgitate();
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
            distanceAndAngle = limelight.getAdjustedDistanceAndAngleToTarget();
            distance = distanceAndAngle[0];
            SmartDashboard.putNumber("Distance", distance);
            angle = distanceAndAngle[1];
            // aligned = false;
            turn = 0;
            forward = 0;
            if (joystick1.getRawButton(B_ALIGN) && limelight.sees()) {
                // aligned=alignTo(angle, distance);
                aligned = align();
                if (aligned) {
                    shoot(distance);
                }
            } else if (joystick2.getRawButton(B_SHOOT)) {
                shoot(distance);// There will be a function based on ll to find this
            } else {
                stop();
                aligned = false;
            }
            if (angle > 0) {
                turn *= -1;
            }
            // hood.setAngle(20);//There will be a function based on ll to find this
        }
        SmartDashboard.putBoolean("Aligned", aligned);
        hood.update();
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
