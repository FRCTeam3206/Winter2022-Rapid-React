package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallChaser {
    private DifferentialDrive drive;

    public BallChaser(DifferentialDrive drive) {
        this.drive = drive;
        NetworkTable camTable = NetworkTableInstance.getDefault().getTable("photonvision")
                .getSubTable("Microsoft_LifeCam_HD-3000");
        if (DriverStation.getAlliance() == Alliance.Red) {
            camTable.getEntry("pipelineIndex").setNumber(1);
        } else {
            camTable.getEntry("pipelineIndex").setNumber(0);
        }
    }

    public boolean chase() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision")
                .getSubTable("Microsoft_LifeCam_HD-3000");
        if (!table.getEntry("hasTarget").getBoolean(false))
            return true;
        double angle = table.getEntry("targetYaw").getDouble(0);
        double turn = -(angle) / 30 * .5;
        double d = Math.sqrt(table.getEntry("targetArea").getDouble(1000000) / 100);
        double distance = 4.75
                / (Math
                        .tan(d/2*(58.4/180*Math.PI)))
                / 12;
        SmartDashboard.putNumber("Ball Dist", distance);
        double forward = Math.sqrt((distance - .8) / 5);
        // drive.arcadeDrive(-forward, turn);
        return distance < 1;
    }
}