// This is a class to be imported for chronos-drive. It contains all that's needed to operate the climber

// If the phoenix packages are needed, use this link to import them: https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2022-latest.json

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

class chronosClimber {
    // Port / channel definitions
    private final int JOY_PORT;
    private final int MOTOR1_PORT, MOTOR2_PORT;

    // Climber speed increment
    private final double SPEED_INC = 0.1;

    // Climber button layout
    private final int CONTROLSCHEME[] = {0, 0, 0, 0}; // This being "final" means nothing because the values can be changed after initalization

    // Define all the needed objects
    private Joystick _Climber;
    private VictorSPX _Motor1;
    private VictorSPX _Motor2;

    // Class variables for ease of use
    private boolean climberUp, climberDown;

    // Climb speed. 1 for full power, 0 for no power, negative numbers to flip controls
    private double climbSpeed;
    private final double DEFAULT_CLIMBSPEED;

    // Initialize values, create objects, and get a default climber speed
    public chronosClimber(int joystickPort, int motorPort1, int motorPort2, int buttons[], double speed) {
        // Constructor breakdown:
            // int joystickPort - The port/channel the joystick is connected to
            // int motorPort1 - The port for 1 of 2 motors
            // int motorPort2 - The port for 2 of 2 motors
            // int buttons[4] - An array that holds the button combos for speed control
                    // buttons[0] - maps to which button increases speed
                    // buttons[1] - maps to which button decreases speed
                    // buttons[2] - maps to which button sets the speed to default
                    // buttons[3] - maps to which button disables the climber entirely
            // double speed - The default and starting speed for the climber. Use values between 0 and 1


        JOY_PORT = joystickPort;
        MOTOR1_PORT = motorPort1;
        MOTOR2_PORT = motorPort2;

        for(int i = 0; i < 4; i++) {
            CONTROLSCHEME[i] = buttons[i];
        }

        climberUp = false; 
        climberDown = false;

        if(speed <= 1 && speed >= 0) {
            DEFAULT_CLIMBSPEED = speed;
            climbSpeed = DEFAULT_CLIMBSPEED;
        } else {
            edu.wpi.first.wpilibj.DriverStation.reportWarning("Incorrect climb speed set. Enter a speed between 0 and 1 (inclusive)", false);
            edu.wpi.first.wpilibj.DriverStation.reportWarning("DEFAULT_CLIMBSPEED set to 0 as failsafe. Please initalize the class with a proper speed value", false);
            
            DEFAULT_CLIMBSPEED = 0;
            climbSpeed = DEFAULT_CLIMBSPEED;
        }

        _Climber = new Joystick(JOY_PORT);
        _Motor1 = new VictorSPX(MOTOR1_PORT);
        _Motor2 = new VictorSPX(MOTOR2_PORT);

        _Motor2.follow(_Motor1);
    }

    // Set the climber speed and make sure the passed value isn't out of bounds
    public void setClimbSpeed(double speed) {
        if(speed > 1 || speed < 0) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning("Incorrect climb speed set. Enter a speed between 0 and 1 (inclusive)", false);
            return;
        }
        climbSpeed = speed;
    }

    // Return the climber speed
    public double getClimbSpeed() {
        return climbSpeed;
    }

    public void resetClimbSpeed() {
        climbSpeed = DEFAULT_CLIMBSPEED;
    }

    // The actual climb function itself
    protected void climb() {
        climberUp = _Climber.getTop();
        climberDown = _Climber.getTrigger();

        // Climb speed control
        handleClimbSpeed();

        // If the top button is pressed, move the climber up
        if(climberUp && !climberDown) {
            _Motor1.set(ControlMode.PercentOutput, climbSpeed);
        }

        // If the trigger is pressed, move the climber down
        if(climberDown && !climberUp) {
            _Motor1.set(ControlMode.PercentOutput, -climbSpeed);
        }

        // If neither button is pressed, or both button are pressed, don't move the motors
        if((!climberDown && !climberUp) || (climberUp && climberDown)) {
            _Motor1.set(ControlMode.PercentOutput, 0);
        }
    }

    protected void handleClimbSpeed() {
        // These buttons are located on the top of the joystick, where your thumb would go
        int speedUp = CONTROLSCHEME[0];
        int speedDown = CONTROLSCHEME[1];
        int defaultSpeed = CONTROLSCHEME[2];
        int noSpeed = CONTROLSCHEME[3];


        // If the top-left button of the joystick has been pressed, increase speed
        if(_Climber.getRawButton(speedUp) && _Climber.getRawButtonReleased(speedUp)) {
            setClimbSpeed(climbSpeed + SPEED_INC);
        }

        // If the top-right button of the joystick has been pressed, decrease speed
        if(_Climber.getRawButton(speedDown) && _Climber.getRawButtonReleased(speedDown)) {
            setClimbSpeed(climbSpeed - SPEED_INC);
        }

        // If the bottom-left button of the joystick has been pressed, revert to default speed
        if(_Climber.getRawButton(defaultSpeed) && _Climber.getRawButtonReleased(defaultSpeed)) {
            setClimbSpeed(DEFAULT_CLIMBSPEED);
        }

        // If the bottom-right button of the joystick has been pressed, set the speed to 0
        if(_Climber.getRawButton(noSpeed) && _Climber.getRawButtonReleased(noSpeed)) {
            setClimbSpeed(0);
        }
    }

};