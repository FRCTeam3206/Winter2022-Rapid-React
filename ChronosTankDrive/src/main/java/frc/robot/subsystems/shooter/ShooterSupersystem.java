package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Subsystem;
import static frc.robot.Constants.Buttons.*;

public class ShooterSupersystem extends Subsystem{
    private Shooter shooter;
    private Hood hood;
    private Limelight limelight;
    private DifferentialDrive driveTrain;
    private GenericHID joystick;
    public ShooterSupersystem(Shooter shooter,Hood hood,Limelight limelight,DifferentialDrive driveTrain,GenericHID joystick){
        this.shooter=shooter;
        this.hood=hood;
        this.limelight=limelight;
        this.driveTrain=driveTrain;
        this.joystick=joystick;
    }
    @Override
    public void init() {
        shooter.init();
        hood.init();
    }
    
    // TODO: Review this method carefully. There was a merge conflict between code written on 3/15 and on 3/16 and I
    //       tried to merge them together to maintain both contributions. -CRS
    @Override
    public void periodic() {
        // boolean shooting=false;  // I don't think this is used
        if (joystick.getRawButton(B_SHOOTER_FAILSAFE)) {
            // assumes driver has parked robot against dasher board under hub
            hood.setAngle(12);  // launch angle of 78 deg, found by limited testing tonight
            shooter.shoot(2300); // found by limited testing tonight
            // shooting=true;
        } else {
          double[] distanceAndAngle=limelight.getAdjustedDistanceAndAngleToTarget();
          double distance=distanceAndAngle[0];
          double angle=distanceAndAngle[1];
          boolean aligned=false;
          double distAway=distance-8*12;
          if(joystick.getRawButton(B_ALIGN)){
              double turn=0;
              double forward=0;
              if(Math.abs(angle)>10){
                  turn=.7;
              }else if(Math.abs(angle)<3){
                  turn=.5;
              }else{
                  aligned=true;
              }
              if(angle>0){
                  turn*=-1;
              }
              hood.setAngle(8.8);//There will be a function based on ll to find this
            }
            if(joystick.getRawButton(B_SHOOT)){
              shooter.shoot(2650);//There will be a function based on ll to find this
              
            }else{
              shooter.stop();
            }
          hood.update();
          }
        }
    }
}
//when bumper is against the hub to shoot, hood angle should be 78 degrees, and rpm should be 2300
