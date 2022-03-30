package frc.robot;

public class Constants {
    public static final double GOAL_HEIGHT = 8 + 8 * 12;

    public static class Shooter {
        public static final double sparkmax_kP = 8e-4;
        public static final double sparkmax_kI = 0;
        public static final double sparkmax_kD = 0;
        public static final double sparkmax_kIz = 0;
        public static final double sparkmax_kFF = 0.000175;
        public static final double sparkmax_kMaxOut = 1;
        public static final double sparkmax_kMinOut = -1;

        public static final double hood_kP = 0.64;
        public static final double hood_kI = 0;
        public static final double hood_kD = 0;
        public static final double hood_kF = 0;
        public static final int hood_kIz = 0;
        public static final double hood_kMaxOut = 1;
        public static final long SHOOT_TIME = 2000;
        /**
         * Which PID slot to pull gains from. Starting 2018, you can choose from
         * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
         * configuration.
         */
        public static final int kSlotIdx = 0;

        /**
         * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
         * now we just want the primary one.
         */
        public static final int kPIDLoopIdx = 0;

        /**
         * Set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int kTimeoutMs = 30;

        /* Choose so that Talon does not report sensor out of phase */
        public static boolean kSensorPhase = true;

        /**
         * Choose based on what direction you want to be positive,
         * this does not affect motor invert.
         */
        public static boolean kMotorInvert = false;

        public static final double SHOOTER_HEIGHT_DIFF = 104 - 17.25;// Goal height-Shooter height

        public static double HOOD_ZERO_POS = 6;

        public static double HOOD_TICKS_PER_ROTATION = 40960;  // constant for 10:1 gearbox

        public static double HOOD_TEETH_ANGLE_RAT = 48 / 41.233;

        public static double HOOD_SPOOL_TEETH = 24;

        public static double HOOD_RANGE = 41;
    }

    public static class Buttons {
        public static final int B_INTAKE = 1;
        public static final int B_DEPLOY = 6;
        public static final int B_ALIGN = 2;
        public static final int B_SHOOT = 2;
        public static final int B_REVERSE_INTAKE = 5;
        public static final int B_SHOOTER_FAILSAFE = 7;
        public static final int B_HOME = 10;
    }

    public static class IDS {
        public static final int SHOOT_PORT = 5;
        public static final int INTAKE_MOTOR_PORT = 6;
        public static final int KICKER_PORT = 7;
        public static final int INTAKE_DEPLOY_PORT = 0;
        public static final int HOOD_PORT = 9;
        public static final int HOOD_LIMIT_PORT = 0;
    }

    public static final class Limelight {
        public static final double LIME_HEIGHT = 24;
        public static final double LIME_X_OFFSET = 9.5;
        public static final double LIME_ANGLE_OFFSET = 32 + 3;
        public static final double LIME_Y_OFFSET = 5 + 9.5;
    }

    public static final class Drive {
        public static final boolean TURN_LIMIT = true;
        public static final double TURN_SCALE = 0.5;

    }
}
