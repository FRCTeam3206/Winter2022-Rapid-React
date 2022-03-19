package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {
    public static final double H2=8*12+8;
    public double T0;
    public double H1;
    public double X_OFF=0;
    public double Y_OFF=0;
    private String name="limelight";
    public Limelight(double height,double distance,double angle){
        T0=Math.atan((H2-height)/distance)-angle*Math.PI/180;
        H1=height;
    }
    public Limelight (double height,double angle,double xOffset,double yOffset){
        T0=angle;
        H1=height;
        X_OFF=xOffset;
        Y_OFF=yOffset;
    }
    public void setName(String name){
        this.name=name;
    }
    public String getName(){
        return name;
    }
    private double dist(double y){
        y=y*Math.PI/180;
        return (H2-H1)/(Math.tan(y+T0));
    }
    public double getDistanceFromTarget(){
        return dist(NetworkTableInstance.getDefault().getTable(name).getEntry("ty").getDouble(0.0));
    }
    public double getHorizontalAngleToTarget(){
        return NetworkTableInstance.getDefault().getTable(name).getEntry("tx").getDouble(0.0);
    }
    //This only requires one retrevial of the NetworkTable
    public double[] getDistanceAndAngleToTarget(){
        NetworkTable table=NetworkTableInstance.getDefault().getTable(name);
        return new double[]{dist(table.getEntry("ty").getDouble(0.0)),table.getEntry("tx").getDouble(0.0)};
    }
    public double[] getAdjustedDistanceAndAngleToTarget(){
        NetworkTable table=NetworkTableInstance.getDefault().getTable(name);
        double rawDist=dist(table.getEntry("ty").getDouble(0.0));
        double rawAngle=table.getEntry("tx").getDouble(0.0)*Math.PI/180;
        double realDist=Math.sqrt(rawDist*rawDist+X_OFF*X_OFF-2*rawDist*X_OFF*Math.sin(rawAngle));
        double realAngle=Math.acos(rawDist/realDist*Math.cos(rawAngle))*180/Math.PI;
        SmartDashboard.putNumber("Adjusted Angle", realAngle);
       // return new double[]{realDist+Y_OFF,realAngle};
       return new double[]{0,rawAngle*180/Math.PI};
    }
    public boolean sees(){
        NetworkTable table=NetworkTableInstance.getDefault().getTable(name);
        return table.getEntry("tshort").getDouble(0.0)>.1;
    }
}
