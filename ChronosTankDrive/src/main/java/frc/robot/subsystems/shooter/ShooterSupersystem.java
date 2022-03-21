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
    private GenericHID joystick1,joystick2;
    public Hood getHood(){
        return hood;
    }
    public ShooterSupersystem(Shooter shooter, Hood hood, Limelight limelight, DifferentialDrive driveTrain,
            GenericHID joystick1,GenericHID joystick2) {
        this.shooter = shooter;
        this.hood = hood;
        this.limelight = limelight;
        this.driveTrain = driveTrain;
        this.joystick1 = joystick1;
        this.joystick2=joystick2;
    }

    @Override
    public void init() {
        shooter.init();
        //hood.init();
        SmartDashboard.putNumber("RPM", 0);
    }
    public double hoodAngle(double distance){
        return 90-Math.atan((Constants.Shooter.SHOOTER_HEIGHT_DIFF+Math.sqrt(Math.pow(Constants.Shooter.SHOOTER_HEIGHT_DIFF,2)+Math.pow(distance,2)))/distance)*180/Math.PI;
    }
    public boolean alignTo(double angle,double distance){
        double turn = -angle / 25;
        SmartDashboard.putNumber("Turn", turn);
        if (Math.abs(turn) < .03) {
            turn = 0;
        }
        driveTrain.arcadeDrive(0, turn);
        double hoodAngle=hoodAngle(distance);
        SmartDashboard.putNumber("Hood Angle", hoodAngle);
        hood.setAngle(hoodAngle);
        return turn<.2&&Math.abs(hood.getAngle()-hoodAngle)<2;
    }
    public boolean align(){
        double[] distanceAndAngle = limelight.getAdjustedDistanceAndAngleToTarget();
        double distance = distanceAndAngle[0];
        double angle=distanceAndAngle[1];
        return alignTo(angle, distance);
    }
    public void shoot(double distance){
        shooter.shoot(4.045*distance+2374.7);
    }
    public void shoot(){
        double[] distanceAndAngle = limelight.getAdjustedDistanceAndAngleToTarget();
        double distance=distanceAndAngle[0];
        shoot(distance);
    }
    public void stop(){
        shooter.stop();
    }
    @Override
    public void periodic() {
        // boolean shooting=false; // I don't think this is used
        double[] distanceAndAngle;
        double distance;
        double angle;
        double distAway;
        boolean aligned = false;
        double turn;
        double forward;
        if (joystick2.getRawButton(B_SHOOTER_FAILSAFE)) {
            // assumes driver has parked robot against dasher board under hub
            hood.setAngle(12); // launch angle of 78 deg, found by limited testing tonight
            shooter.shoot(2300); // found by limited testing tonight
            // shooting=true;
        } else {
            distanceAndAngle = limelight.getAdjustedDistanceAndAngleToTarget();
            distance = distanceAndAngle[0];
            angle = distanceAndAngle[1];
            aligned = false;
            turn = 0;
            forward = 0;
            if (joystick1.getRawButton(B_ALIGN) && limelight.sees()) {
                //aligned=alignTo(angle, distance);
                aligned=align();
            }
            if (joystick2.getRawButton(B_SHOOT)) {
                shoot(distance);// There will be a function based on ll to find this
            } else {
                shooter.stop();
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
    }
}

// when bumper is against the hub to shoot, hood angle should be 78 degrees, and
// rpm should be 2300
