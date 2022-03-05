// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private static final Color redBall = new Color(.54,.34,.12);
  private static final Color blueBall = new Color(.16,.40,.47);
  private ColorMatch colorMatch = new ColorMatch();
  @Override
  public void robotInit() {
    colorMatch.addColorMatch(redBall);
    colorMatch.addColorMatch(blueBall);  
    colorMatch.setConfidenceThreshold(.85);  
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  float[] hsv=new float[3];
  int colorMultiplier=255*2;
  @Override
  public void teleopPeriodic() {
    
    Color detected=colorSensor.getColor();
    SmartDashboard.putNumber("Red", detected.red);
    SmartDashboard.putNumber("Green", detected.green);
    SmartDashboard.putNumber("Blue", detected.blue);
    hsv=java.awt.Color.RGBtoHSB((int)(colorSensor.getRed()*colorMultiplier),(int)(colorSensor.getGreen()*colorMultiplier),(int)(colorSensor.getBlue()*colorMultiplier),hsv);
    SmartDashboard.putNumber("Hue",hsv[0]);
    SmartDashboard.putNumber("Sat",hsv[1]);
    SmartDashboard.putNumber("Val",hsv[2]);
    ColorMatchResult matchResult=colorMatch.matchColor(detected);
    String output="No Ball";
    double confidence=-1;
    if(colorMatch.matchColor(detected)!=null&&matchResult.confidence>.75){
      Color result=matchResult.color;
      if(result==redBall){
        output="Red Ball";
      }else if(result==blueBall){
        output="Blue Ball";
      }else{
        output="No Ball";
      }
      confidence=matchResult.confidence;
    }
    SmartDashboard.putString("Output", output);
    SmartDashboard.putNumber("Confidence", confidence);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
