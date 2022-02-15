package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    public static final double H2=8*12+8;
    public double T0;
    public double H1;
    private String name="limelight";
    public Limelight(double height,double distance,double angle){
        T0=Math.atan((H2-height)/distance)-angle*Math.PI/180;
        H1=height;
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
    public double[] getDistanceAndAngleToTarget(){
        NetworkTable table=NetworkTableInstance.getDefault().getTable(name);
        return new double[]{dist(table.getEntry("ty").getDouble(0.0)),table.getEntry("tx").getDouble(0.0)};
    }
}
