package example.robot;

public final class Constants {

    public static final int TEAM_NUMBER = 9999;

    /** true ise Robot.java bir log dosyasini replay eder (sim/gercek yerine). */
    public static final boolean REPLAY_MODE = false;

    public static final class DriveConstants {
        // CAN ID'leri, encoder offsetleri, disli oranlari, drive/steer PID gains
        // artik generated/TunerConstants.java icinde (CTRE Swerve API generated dosyasi).
        // Burada sadece surus komutlarinin (TeleopDriveCommand, AlignShootCommand) kullandigi
        // hiz limitleri kaliyor.
        public static final double MAX_SPEED_MPS   = 4.69;      // m/s - TunerConstants.kSpeedAt12Volts ile ayni
        public static final double MAX_ANGULAR_RPS = 2 * Math.PI; // rad/s
    }

    public static final class VisionConstants {
        public static final String LIMELIGHT_NAME = "limelight";

        public static final double CAMERA_HEIGHT_METERS = 0.50;
        public static final double CAMERA_PITCH_DEG     = 30.0;
        public static final double TARGET_HEIGHT_METERS = 1.83;

        // tx tabanli rotasyon P kazanci titiriyorsa azalt
        public static final double AIM_KP           = 0.04;
        public static final double AIM_TOLERANCE_DEG = 1.0;

        // MegaTag2 fuzyonu icin maks izin verilen yaw hizi (derece/s)
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

        // velocity pid + ff - rotor rps cinsinden, tune et
        public static final double FLYWHEEL_KP = 0.1;
        public static final double FLYWHEEL_KI = 0.0;
        public static final double FLYWHEEL_KD = 0.0;
        public static final double FLYWHEEL_KS = 0.1;   // statik surtunme (v)
        public static final double FLYWHEEL_KV = 0.12;  // her rps basina voltaj (v/rps)

        public static final double SHOOT_RPS           = 80.0; // hedef flywheel hizi - tune et
        public static final double SPEED_TOLERANCE_RPS =  3.0;

        public static final double FEEDER_SPEED         =  0.8;
        public static final double FEEDER_REVERSE_SPEED = -0.4; // sikisma acma

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

        // intake saat yonunde aciliyor (deploy = pozitif rotasyon) -> Clockwise_Positive
        public static final boolean PIVOT_INVERTED  = true;

        // pivot position pid (mekanizma rotasyonu cinsinden)
        public static final double PIVOT_KP         = 3.0;
        public static final double PIVOT_KI         = 0.0;
        public static final double PIVOT_KD         = 0.0;
        // TODO: max planetary 50:1 kutu direkt mile bagliysa 50.0 yap; aradaki zincir/disli
        // reduksiyonuysa gercek toplam orani (50 * ek reduksiyon) gir - 36.0 dogrulanmadi
        public static final double PIVOT_GEAR_RATIO = 36.0;

        // pivot konumlari (mekanizma rotasyonu) - tune et
        public static final double PIVOT_STOWED_ROT    =  0.0;
        public static final double PIVOT_DEPLOYED_ROT  = 10.0;
        public static final double PIVOT_TOLERANCE_ROT =  0.3;

        public static final int PIVOT_SUPPLY_LIMIT  = 30;
        public static final int PIVOT_STATOR_LIMIT  = 60;

        // roborio dijital giris portu (beam break sensoru)
        public static final int BEAM_BREAK_DIO = 0;

        // intake uzerindeki alma rulosu - kraken x44 (LT basili tutunca doner)
        public static final int ROLLER_ID = 27;

        public static final boolean ROLLER_INVERTED = false;

        public static final double ROLLER_INTAKE_SPEED = 0.8; // tune et

        public static final int ROLLER_SUPPLY_LIMIT = 30;
        public static final int ROLLER_STATOR_LIMIT = 40; // x44 - x60'tan dusuk limit
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT   = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double DEADBAND = 0.05;
    }

    public static final class AutoConstants {
        // PathPlanner translasyon PID (m/s hatasi -> m/s cikis)
        public static final double TRANSLATION_KP = 5.0;
        // PathPlanner rotasyon PID (rad hatasi -> rad/s cikis)
        public static final double ROTATION_KP = 5.0;
    }

}
