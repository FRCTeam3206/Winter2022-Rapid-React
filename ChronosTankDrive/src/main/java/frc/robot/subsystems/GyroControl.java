package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroControl {
    ADIS16448_IMU imu;
    DifferentialDrive drive;
    double zeroPos = 0;

    public GyroControl(DifferentialDrive drive) {
        imu = new ADIS16448_IMU();
        this.drive = drive;
    }

    public void zero() {
        imu.reset();
    }

    public double getAngle() {
        SmartDashboard.putNumber("rotX", imu.getGyroAngleX());
        SmartDashboard.putNumber("rotY", imu.getGyroAngleY());
        SmartDashboard.putNumber("rotZ", imu.getGyroAngleZ());
        double angle = Math.floorMod((long) (imu.getGyroAngleX() - zeroPos), 360l);
        return (angle);
    }

    public boolean turnTo(double angle) {
        double angleDiff = getAngle() - angle;
        drive.arcadeDrive(0, angleDiff / 60);
        return angleDiff / 60 < .2;
    }
}
