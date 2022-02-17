// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
//import edu.wpi.first.wpilibj.DriverStation; // Idk why this is marked as unused, but I'm very clearly using it in reportWarning()

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Motor channel definitions
  private final int SPX0_CHANNEL = 0, SPX1_CHANNEL = 1, SPX2_CHANNEL = 2, SPX3_CHANNEL = 3;
  private final int INTAKE_CHANNEL = 4;

  // Controller channel definitions (Note: Different from motor channels)
  private final int XBOX_CHANNEL = 1, JOYSTICK_CHANNEL = 0;

  // Controller objects
  private XboxController weapons = new XboxController(XBOX_CHANNEL);
  private Joystick _Joystick = new Joystick(JOYSTICK_CHANNEL);

  // Motor objects
  private VictorSP intake = new VictorSP(INTAKE_CHANNEL);
  private VictorSPX _Spx0 = new VictorSPX(SPX0_CHANNEL);
  private VictorSPX _Spx1 = new VictorSPX(SPX1_CHANNEL);
  private VictorSPX _Spx2 = new VictorSPX(SPX2_CHANNEL);
  private VictorSPX _Spx3 = new VictorSPX(SPX3_CHANNEL);

  // Controller inverts
  private final boolean JOY_XINVERT = false;
  private final boolean JOY_YINVERT = true;
  private final boolean JOY_ZINVERT = false;

  // Controller deadzones
  private final double JOY_DEADZONE = 0.30, XBOX_DEADZONE = 0.10;

  // Controller speed mods
  private final double JOY_MOD = 0.50, XBOX_MOD = 0.50;

  // Class variable definitions
  private int modeToggle = 0;

  protected void reportWarning(String msg) {
    // I don't want to type out edu.wpi.first.wpilibj.DriverStation every single time I want a debug message
    edu.wpi.first.wpilibj.DriverStation.reportWarning(msg, false);

  }

  protected void buttonMotors(XboxController controller, VictorSP motor, char button, double speed) {
    // This function takes a controller object, motor object, button, and speed, and checks if the button has been pressed since last check
    // If so, set the motor's speed to the given speed

    boolean buttonPressed = false;
    boolean buttonWasPressed = false;
    
    switch(button) {
      case('x'):
        buttonPressed = controller.getXButton();
        buttonWasPressed = controller.getXButtonReleased();

      break;
      case('y'):
        buttonPressed = controller.getYButton();
        buttonWasPressed = controller.getYButtonReleased();

      break;
      case('a'):
        buttonPressed = controller.getAButton();
        buttonWasPressed = controller.getAButtonReleased();

      break;
      case('b'):
        buttonPressed = controller.getBButton();
        buttonWasPressed = controller.getBButtonReleased();

      break;
      default:
        String msg = "Wrong button input given";
        System.out.println(msg);
        reportWarning(msg);
    }

    if(buttonPressed && buttonWasPressed) {
      motor.set(speed);

    }
  }

  protected double getStick(XboxController controller, char stick, char axis) {
    // Get the value of the specified stick axis 
    switch(stick) {
      case('l'):
        if(axis == 'x') {
          return controller.getLeftX();

        } else if(axis == 'y') {
          return controller.getLeftY();

        } else {
          reportWarning("Incorrect axis. Please use either 'x' or 'y'");
          return 255;

        }
      case('r'):
        if(axis == 'x') {
          return controller.getRightX();

        } else if(axis == 'y') {
          return controller.getRightY();

        } else {
          reportWarning("Incorrect axis. Please use either 'x' or 'y'");
          return 255;

        }
      default:
      reportWarning("Incorrect stick. Please use either 'l' or 'r'");
      return 255;

    }
  }

  protected int btoi(boolean a) {
    // This is dumb. I shouldn't need to make this function. Java should just have an explicit conversion
    return (a) ? 1 : 0;
  }

  protected int controlMode(XboxController controller, int toggle) {
    boolean bumperPressed = controller.getLeftBumper();
    boolean bumperWasPressed = controller.getLeftBumperPressed();

    double stick = getStick(controller, 'l', 'y');
    boolean stickMoved = (stick >= XBOX_DEADZONE || stick <= -XBOX_DEADZONE);

    if(bumperPressed && bumperWasPressed) {
      if(toggle == 0) {
        toggle = 1;
      } else {
        toggle = 0;
      }
    }

    if(toggle == 0) {
      // Standard controls
      buttonMotors(weapons, intake, 'a', 0);
      buttonMotors(weapons, intake, 'y', 1);
      return 0;
    } else if(toggle == 1) {
      buttonMotors(weapons, intake, 'x', applyStickMod(stick, XBOX_MOD, btoi(stickMoved)));
      return 1;
    } else {
      return 255;
    }
  }

  protected double getJoysticks(Joystick joy, char axis) {
    // Return the stick values from a joystick object
    switch(axis) {
      case('x'):
        // This is basically a condensed if statement. If JOY_XINVERT equals true, return an inverted x axis. Otherwise, return the normal axis
        return (JOY_XINVERT) ? joy.getX() * -1 : joy.getX();
      case('y'):
        return (JOY_YINVERT) ? joy.getY() * -1 : joy.getY();
      case('z'):
        return (JOY_ZINVERT) ? joy.getZ() * -1 : joy.getY();
      default:
        reportWarning("[getJoysticks] Invalid axis used in the getJoysticks function (Code 128)");
        return 128;
    }
  }

  protected double applyStickMod(double stickVal, double stickMod, double speedMod) {
    return stickVal * stickMod * speedMod;
  }

  protected double constrainRange(double stickVal, double minVal, double maxVal, double lowerBound, double upperBound) {
    // Return the fine conversion value of a variable linear transformation
    // Credit: Simone on StackExchange: https://stats.stackexchange.com/a/178629
    double result = (upperBound - lowerBound) * ((stickVal - minVal) / (maxVal - minVal)) + lowerBound;
    return (result > upperBound || result < lowerBound ||  Double.isNaN(result) == true) ? 0 : result; // Check if the values aren't crazy.
  }

  protected void voidDrive(Joystick joy) {
    double stickX, stickY, throttle;
    boolean priorityForward, priorityBack, priorityLeft, priorityRight;

    throttle = constrainRange(joy.getThrottle() * -1, -1, 1, 0, 1);
    // The documentation doesn't say it, but the throttle can range between -1 and 1. So this takes the value and constrains it between 0 and 1
      // I could do math to simplify the equation, but I'm lazy and I've already made a function that does it for me

    stickX = applyStickMod(getJoysticks(joy, 'x'), JOY_MOD, throttle);
    stickY = applyStickMod(getJoysticks(joy, 'y'), JOY_MOD, throttle);

    priorityForward = (stickY >= 0.7);
    priorityBack = (stickY <= -0.7);
    priorityLeft = (stickX <= -0.7);
    priorityRight = (stickX >= 0.7);

    if(stickY >= JOY_DEADZONE && (!priorityRight && !priorityLeft) || priorityForward) {
      // Move forward
      _Spx0.set(ControlMode.PercentOutput, -stickY);
      _Spx1.set(ControlMode.PercentOutput, stickY);

    } else if(stickX >= JOY_DEADZONE && (!priorityForward && !priorityBack) || priorityRight) {
      // Turn Right
      _Spx0.set(ControlMode.PercentOutput, -stickX);
      _Spx1.set(ControlMode.PercentOutput, -stickX);

    } else if(stickX <= -JOY_DEADZONE && (!priorityForward && !priorityBack) || priorityLeft) {
      // Turn Left
      _Spx0.set(ControlMode.PercentOutput, -stickX);
      _Spx1.set(ControlMode.PercentOutput, -stickX);

    } else if(stickY <= -JOY_DEADZONE && (!priorityRight && !priorityLeft) || priorityBack) {
      // Move backward
      _Spx0.set(ControlMode.PercentOutput, -stickY);
      _Spx1.set(ControlMode.PercentOutput, stickY);

    } else {
      // Stop
      _Spx0.set(ControlMode.PercentOutput, 0);
      _Spx1.set(ControlMode.PercentOutput, 0);

    }

  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    _Spx2.follow(_Spx1);
    _Spx3.follow(_Spx0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    modeToggle = controlMode(weapons, modeToggle);
    if(modeToggle == 255) {
      reportWarning("Function \"controlMode\" returned an error");
      modeToggle = 0;
    }
        
    voidDrive(_Joystick);

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

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
