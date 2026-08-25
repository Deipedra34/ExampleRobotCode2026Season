package example.robot;

public final class Constants {

    public static final int TEAM_NUMBER = 9999;

    /** if true, Robot.java replays a log file instead of running for real/sim. */
    public static final boolean REPLAY_MODE = false;

    public static final class DriveConstants {
        // CAN ids, encoder offsets, gear ratios, drive/steer PID gains all live in
        // generated/TunerConstants.java now (the CTRE swerve API generated file).
        // this just keeps the speed caps that the drive commands (TeleopDriveCommand,
        // AlignShootCommand) actually need.
        public static final double MAX_SPEED_MPS   = 4.69;      // m/s, same as TunerConstants.kSpeedAt12Volts
        public static final double MAX_ANGULAR_RPS = 2 * Math.PI; // rad/s
    }

    public static final class VisionConstants {
        public static final String LIMELIGHT_NAME = "limelight";

        public static final double CAMERA_HEIGHT_METERS = 0.50;
        public static final double CAMERA_PITCH_DEG     = 30.0;
        public static final double TARGET_HEIGHT_METERS = 1.83;

        // tx-based rotation P gain, back this off if it's oscillating
        public static final double AIM_KP           = 0.04;
        public static final double AIM_TOLERANCE_DEG = 1.0;

        // max yaw rate we'll trust for MegaTag2 fusion (deg/s)
        public static final double MAX_VISION_YAW_RATE_DEG_PER_SEC = 360.0;
    }

    public static final class ShooterConstants {
        public static final int LEFT_FLYWHEEL_ID   = 21;
        public static final int LEFT_FLYWHEEL_2_ID = 23;
        public static final int RIGHT_FLYWHEEL_ID  = 22;
        public static final int RIGHT_FLYWHEEL_2_ID = 24;
        public static final int FEEDER_ID           = 25;

        public static final boolean LEFT_FLYWHEEL_INVERTED   = false;
        public static final boolean LEFT_FLYWHEEL_2_INVERTED = true;
        public static final boolean RIGHT_FLYWHEEL_INVERTED  = false;
        public static final boolean RIGHT_FLYWHEEL_2_INVERTED = false;
        public static final boolean FEEDER_INVERTED          = true;

        // velocity PID + FF, units are rotor rps, still needs tuning
        public static final double FLYWHEEL_KP = 0.1;
        public static final double FLYWHEEL_KI = 0.0;
        public static final double FLYWHEEL_KD = 0.0;
        public static final double FLYWHEEL_KS = 0.1;   // static friction (volts)
        public static final double FLYWHEEL_KV = 0.12;  // volts per rps

        public static final double SHOOT_RPS           = 80.0; // target flywheel speed, needs tuning
        public static final double SPEED_TOLERANCE_RPS =  3.0;

        public static final double FEEDER_SPEED         =  0.8;
        public static final double FEEDER_REVERSE_SPEED = -0.4; // for clearing jams

        public static final int FLYWHEEL_SUPPLY_LIMIT = 40;
        public static final int FLYWHEEL_STATOR_LIMIT = 80;
        public static final int FEEDER_SUPPLY_LIMIT   = 30;
    }

    public static final class FeederConstants {
        public static final int ROLLER_ID = 29;

        public static final boolean ROLLER_INVERTED = true;

        public static final double ROLLER_INTAKE_SPEED  =  0.9;
        public static final double ROLLER_OUTTAKE_SPEED = -0.6;

        public static final int ROLLER_SUPPLY_LIMIT = 30;
        public static final int ROLLER_STATOR_LIMIT = 60;
    }

    public static final class IntakeConstants {
        public static final int PIVOT_ID  = 26; // kraken x60

        // intake opens clockwise (deploy = positive rotation) -> Clockwise_Positive
        public static final boolean PIVOT_INVERTED  = true;

        // pivot position PID, units are mechanism rotations
        public static final double PIVOT_KP         = 3.0;
        public static final double PIVOT_KI         = 0.0;
        public static final double PIVOT_KD         = 0.0;
        // TODO: if the 50:1 planetary is direct-driving the shaft this should be 50.0;
        // if there's a chain/gear reduction after it, use the real total ratio
        // (50 * whatever that extra reduction is) - 36.0 has never actually been verified
        public static final double PIVOT_GEAR_RATIO = 36.0;

        // pivot setpoints (mechanism rotations), needs tuning
        public static final double PIVOT_STOWED_ROT    =  0.0;
        public static final double PIVOT_DEPLOYED_ROT  = 10.0;
        public static final double PIVOT_TOLERANCE_ROT =  0.3;

        public static final int PIVOT_SUPPLY_LIMIT  = 30;
        public static final int PIVOT_STATOR_LIMIT  = 60;

        // roboRIO DIO port for the beam break sensor
        public static final int BEAM_BREAK_DIO = 0;

        // intake's own roller, kraken x44 (spins while LT is held)
        public static final int ROLLER_ID = 27;

        public static final boolean ROLLER_INVERTED = false;

        public static final double ROLLER_INTAKE_SPEED = 0.8; // needs tuning

        public static final int ROLLER_SUPPLY_LIMIT = 30;
        public static final int ROLLER_STATOR_LIMIT = 40; // x44, so lower than the x60 limit
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT   = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double DEADBAND = 0.05;
    }

    public static final class AutoConstants {
        // PathPlanner translation PID (m/s error -> m/s output)
        public static final double TRANSLATION_KP = 5.0;
        // PathPlanner rotation PID (rad error -> rad/s output)
        public static final double ROTATION_KP = 5.0;
    }

}
