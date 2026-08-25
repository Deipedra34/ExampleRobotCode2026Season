package example.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

import example.robot.subsystems.drive.CommandSwerveDrivetrain;

/**
 * CTRE swerve API constants for the SDS MK5n modules (Kraken X60 drive, Kraken X44 steer,
 * CANcoder). These are carried straight over from the calibrated values that used to live
 * in the old hand-rolled SwerveModule.java - CAN ids, encoder offsets, gear ratios, current
 * limits, all the same.
 *
 * <p>The drive Slot0 gains here (kP=1.5, kV=2.55804) are kept UN-scaled on purpose -
 * SwerveModuleConstantsFactory converts them to mechanism (m/s) units internally using the
 * gear ratio + wheel radius. The old Constants.DriveConstants.DRIVE_KP/DRIVE_KV were these
 * same raw numbers hand-converted to rotor rps - don't convert them again here.
 */
public class TunerConstants {
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(62.83).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(1.5).withKI(0).withKD(0)
        .withKS(0.025).withKV(2.55804);

    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // falls back to RemoteCANcoder automatically without a Pro license (same as the old RemoteCANcoder behavior)
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // slip current = old DRIVE_STATOR_CURRENT_LIMIT
    private static final Current kSlipCurrent = Amps.of(120);

    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(70))
                .withSupplyCurrentLimitEnable(true)
        );
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(40))
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(50))
                .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    private static final Pigeon2Configuration pigeonConfigs = null;

    // the roboRIO's own native CAN bus - this robot doesn't run a CANivore
    public static final CANBus kCANBus = new CANBus("");

    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.69);

    // the old code never modeled azimuth-drive coupling, so this stays 0 to keep behavior the same.
    // TODO: measure the SDS MK5n's actual coupling ratio and put it here (roughly 50/14).
    private static final double kCoupleRatio = 0.0;

    private static final double kDriveGearRatio = 7.03125;
    private static final double kSteerGearRatio = 26.09090909090909;
    private static final Distance kWheelRadius = Inches.of(2.0);

    private static final boolean kInvertLeftSide = true;
    private static final boolean kInvertRightSide = false;

    private static final int kPigeonId = 35;

    // only used in simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.035);
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 10;
    private static final int kFrontLeftSteerMotorId = 11;
    private static final int kFrontLeftEncoderId = 30;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.031494);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(10.6625);
    private static final Distance kFrontLeftYPos = Inches.of(11.3515);

    // Front Right
    private static final int kFrontRightDriveMotorId = 16;
    private static final int kFrontRightSteerMotorId = 17;
    private static final int kFrontRightEncoderId = 34;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(0.136230);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(10.6625);
    private static final Distance kFrontRightYPos = Inches.of(-11.3515);

    // Back Left
    private static final int kBackLeftDriveMotorId = 14;
    private static final int kBackLeftSteerMotorId = 15;
    private static final int kBackLeftEncoderId = 32;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.485352);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-10.6625);
    private static final Distance kBackLeftYPos = Inches.of(11.3515);

    // Back Right
    private static final int kBackRightDriveMotorId = 12;
    private static final int kBackRightSteerMotorId = 13;
    private static final int kBackRightEncoderId = 31;
    private static final Angle kBackRightEncoderOffset = Rotations.of(-0.339111);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-10.6625);
    private static final Distance kBackRightYPos = Inches.of(-11.3515);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
        );

    /** should only be called once, from RobotContainer. */
    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        );
    }

    /** swerve drive base class built on CTR Electronics' Phoenix 6 API. */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        }

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
    }
}
