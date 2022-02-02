// Copyright (c) 2022 FRC 3026
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;
import edu.wpi.first.wpilibj.DriverStation;

public class DriveAxis {

    String name;

    private double deadzone = 0.1;
    private double axisMod = 1;
    private int accelCoefficient = 8;
    private boolean debug = false;

    private boolean inverted = false;
    private double value = 0;
    private double lastValue = 0;
    private double powValue = 0;
    private double lastPowValue = 0;

    // Use default values for deadzone, axisMod, and accelCoefficient
    public DriveAxis(String name, boolean inverted) {
        this.name = name;
        this.inverted = inverted;
    }

    // Use default values for deadzone, axisMod, and accelCoefficient
    public DriveAxis(String name, boolean inverted, boolean debug) {
        this.name = name;
        this.inverted = inverted;
        this.debug = debug;
    }

    public DriveAxis(String name, boolean inverted, boolean debug, double deadzone, double axisMod, int accelCoefficient) {
        this.name = name;
        this.inverted = inverted;
        this.debug = debug;
        this.deadzone = deadzone;
        this.axisMod = axisMod;
        this.accelCoefficient = accelCoefficient;
    }

    public String toString() {
        return "DriveAxis " + this.name;
    }

    void setValue(double value) {
        int invertVal = (this.inverted) ? -1 : 1;

        this.lastValue = this.value;
        this.value = value * this.axisMod * invertVal;
    }

    double getValue(){
        return this.value;
    }

    boolean isMeasurable() {
        return (Math.abs(this.value) >= this.deadzone);
    }

    // Connor: This formula is simplified from: 1/k * input + (k - 1)/k * old motor pow = new motor pow
    private double getAccelValue() {
        double accelValue = this.value + (this.lastPowValue * this.accelCoefficient - this.lastPowValue) / this.accelCoefficient;

        if (debug) {
            DriverStation.reportWarning(this + " Accel Value: " + accelValue, false);
        }

        return accelValue;
    }

    // TODO: Fix this... WAY wrong
    private double getFine(double axisTotal) {
        double fine = this.value * this.value / axisTotal;

        if (debug) {
            DriverStation.reportWarning(this + " Fine: " + fine, false);
        }

        return fine;
    }

    //
    double getPowValue(double axisTotal) {
        this.lastPowValue = this.powValue;
        this.powValue = (this.isMeasurable()) ? this.getAccelValue() * this.getFine(axisTotal) : 0;

        if (debug) {
            DriverStation.reportWarning(this + " Pow Value: " + this.powValue, false);
        }

        return this.powValue;
    }
}
