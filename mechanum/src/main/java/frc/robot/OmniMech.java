// Copyright (c) FIRST and other WPILib contributors.
// Copyright (c) 2022 FRC 3026
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.System;
import java.lang.Math;
import java.util.HashMap;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class OmniMech extends TimedRobot {

  // Motor properties
  private static final int kFrontLeftChannel = 0;
  private static final int kRearLeftChannel = 2;
  private static final int kFrontRightChannel = 1;
  private static final int kRearRightChannel = 3;

  private VictorSP frontLeft = new VictorSP(kFrontLeftChannel);
  private VictorSP rearLeft = new VictorSP(kRearLeftChannel);
  private VictorSP frontRight = new VictorSP(kFrontRightChannel);
  private VictorSP rearRight = new VictorSP(kRearRightChannel);

  private static final int kDriveChannel = 0;
  private MechanumXboxController driveController;

  private static boolean debug = false;

  @Override
  public void robotInit() {
    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontLeft.setInverted(true);
    rearLeft.setInverted(true);

    this.driveController = new MechanumXboxController(kDriveChannel);
    // OR with debug logging specified
    //this.driveController = new MechanumXboxController(kDriveChannel, debug);
    // or override default values for coefficients
    //this.driveController = new MechanumXboxController(kDriveChannel, debug, deadzone, stickMod, accelCoefficient);
  }

  @Override
  public void teleopPeriodic() {
    HashMap<String, Double> powerValues;

    this.driveController.updateAllAxis();
    powerValues = driveController.getPowerValues();

    frontRight.set(powerValues.get("frontRight"));
    frontLeft.set(powerValues.get("frontLeft"));
    rearRight.set(powerValues.get("rearRight"));
    rearLeft.set(powerValues.get("rearLeft"));
  }
}
