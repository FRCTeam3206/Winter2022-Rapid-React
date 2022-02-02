// Copyright (c) 2022 FRC 3026
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import java.util.HashMap;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;

public class MechanumXboxController extends XboxController {

    private double axisTotal = 0.0;
    private boolean debug = false;
    DriveAxis forward;
    DriveAxis strafe;
    DriveAxis spin;

    /**
     * In omnibot, include this class by specifying the controller channel, the deadzone, and accelleration co-efficient.
     **/
    public MechanumXboxController(int channel) {
        super(channel);
        this.forward = new DriveAxis("forward", true, debug);
        this.strafe = new DriveAxis("strafe", false, debug);
        this.spin = new DriveAxis("spin", false, debug);
    }

        public MechanumXboxController(int channel, boolean debug) {
        super(channel);
        this.forward = new DriveAxis("forward", true, debug);
        this.strafe = new DriveAxis("strafe", false, debug);
        this.spin = new DriveAxis("spin", false, debug);
        }

    public MechanumXboxController(int channel, boolean debug, double deadzone, double stickMod, int accelCoefficient) {
        super(channel);
        this.forward = new DriveAxis("forward", true, debug, deadzone, stickMod, accelCoefficient);
        this.strafe = new DriveAxis("strafe", false, debug, deadzone, stickMod, accelCoefficient);
        this.spin = new DriveAxis("spin", false, debug, deadzone, stickMod, accelCoefficient);
    }

    // Used in power value calculation
    public void updateAxisTotal() {
        double forward = (this.forward.isMeasurable()) ? Math.abs(this.forward.getValue()) : 0;
        double strafe = (this.strafe.isMeasurable()) ? Math.abs(this.strafe.getValue()) : 0;
        double spin = (this.spin.isMeasurable()) ? Math.abs(this.spin.getValue()) : 0;
        this.axisTotal = forward + strafe + spin;
    }

    // Get current state of all drive axis
    public void updateAllAxis() {
        this.forward.setValue(this.getLeftY());
        this.strafe.setValue(this.getLeftX());
        this.spin.setValue(this.getRightX());

        this.updateAxisTotal();
    }

    // Get the power values for the motors
    public HashMap<String, Double> getPowerValues() {
        HashMap<String, Double> powerValues = new HashMap<String, Double>();
        Double forward = this.forward.getPowValue(this.axisTotal);
        Double strafe = this.strafe.getPowValue(this.axisTotal);
        Double spin = this.spin.getPowValue(this.axisTotal);

        powerValues.put("frontRight", forward - strafe - spin);
        powerValues.put("frontLeft", forward + strafe + spin);
        powerValues.put("rearRight", forward + strafe - spin);
        powerValues.put("rearLeft", forward - strafe + spin);

        return powerValues;
    }
}
