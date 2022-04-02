package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Limelight {
    public static final double H2 = 8 * 12 + 8;
    public double T0;
    public double H1;
    public double X_OFF = 0;
    public double Y_OFF = 0;
    private String name = "limelight";

    public Limelight(double height, double distance, double angle) {
        T0 = Math.atan((H2 - height) / distance) - angle * Math.PI / 180;
        H1 = height;
    }

    public Limelight(double height, double angle, double xOffset, double yOffset) {
        T0 = angle * Math.PI / 180;
        H1 = height;
        X_OFF = xOffset;
        Y_OFF = yOffset;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    private double dist(double y) {
        y = y * Math.PI / 180;
        return (H2 - H1) / (Math.tan(y + T0));
    }

    public double getDistanceFromTarget() {
        return dist(NetworkTableInstance.getDefault().getTable(name).getEntry("ty").getDouble(0.0));
    }

    public double getHorizontalAngleToTarget() {
        return NetworkTableInstance.getDefault().getTable(name).getEntry("tx").getDouble(0.0);
    }

    // This only requires one retrevial of the NetworkTable
    public double[] getDistanceAndAngleToTarget() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        return new double[] { dist(table.getEntry("ty").getDouble(0.0)), table.getEntry("tx").getDouble(0.0) };
    }

    public double[] getAdjustedDistanceAndAngleToTarget() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        double ty = table.getEntry("ty").getDouble(0.0) + (table.getEntry("tvert").getDouble(0.0) / 320 * 20.5 / 2);
        double rawDist = dist(ty);
        double rawAngle = table.getEntry("tx").getDouble(0.0) * Math.PI / 180;
        double realDist = Math.sqrt(rawDist * rawDist + X_OFF * X_OFF - 2 * rawDist * X_OFF * Math.sin(rawAngle));
        double realAngle = (rawAngle + Math.atan(X_OFF / rawDist)) * 180 / Math.PI;
        SmartDashboard.putNumber("Distance", (rawDist + Y_OFF));
        SmartDashboard.putNumber("Adjusted Angle", realAngle);
        // return new double[]{realDist+Y_OFF,realAngle};
        return new double[] { rawDist + Y_OFF, realAngle };
    }

    public boolean sees() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        return table.getEntry("tlong").getDouble(0.0) > .1;
    }
}
