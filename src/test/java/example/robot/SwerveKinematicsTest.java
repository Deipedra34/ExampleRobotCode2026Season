package example.robot;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

/**
 * swerve kinematik dönüşümlerini test eder
 * WPILib math sınıfları HAL gerektirmez
 */
class SwerveKinematicsTest {

    // Constants.DriveConstants tan kopyaladım robot geometrisi
    private static final double TRACK  = Units.inchesToMeters(23.5);
    private static final double BASE   = Units.inchesToMeters(23.5);
    private static final double MAX_V  = 4.5; // m/s

    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d( BASE / 2,  TRACK / 2),  // FL
        new Translation2d( BASE / 2, -TRACK / 2),  // FR
        new Translation2d(-BASE / 2,  TRACK / 2),  // BL
        new Translation2d(-BASE / 2, -TRACK / 2)   // BR
    );

    @Test
    void forwardDrive_allModulesPointForward() {
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(
            new ChassisSpeeds(1.0, 0, 0)
        );
        for (SwerveModuleState s : states) {
            assertEquals(0.0, s.angle.getRadians(), 1e-6,
                "Düz ileri sürüşte tüm modüller 0 rad (ileri) bakmalı");
        }
    }

    @Test
    void forwardDrive_allModulesSameSpeed() {
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(
            new ChassisSpeeds(1.5, 0, 0)
        );
        for (SwerveModuleState s : states) {
            assertEquals(1.5, s.speedMetersPerSecond, 1e-6,
                "Düz sürüşte tüm modüller aynı hızda olmalı");
        }
    }

    @Test
    void strafeDrive_allModulesPointLeft() {
        // ySpeed pozitif = sola
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(
            new ChassisSpeeds(0, 1.0, 0)
        );
        for (SwerveModuleState s : states) {
            assertEquals(Math.PI / 2, s.angle.getRadians(), 1e-6,
                "Saf sola sürüşte tüm modüller 90 derece (sol) bakmalı");
        }
    }

    @Test
    void desaturate_exceedingMaxSpeed_clampsToMax() {
        // 10 m/s gibi bir değer işte desaturate MAX_V ye indirmeli
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(
            new ChassisSpeeds(10.0, 0, 0)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_V);
        for (SwerveModuleState s : states) {
            assertTrue(s.speedMetersPerSecond <= MAX_V + 1e-6,
                "Desaturate sonrası hız MAX_SPEED'i geçmemeli");
        }
    }

    @Test
    void desaturate_withinLimit_doesNotChange() {
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(
            new ChassisSpeeds(1.0, 0, 0)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_V);
        for (SwerveModuleState s : states) {
            assertEquals(1.0, s.speedMetersPerSecond, 1e-6,
                "Limit altındaki hız değiştirilmemeli");
        }
    }

    @Test
    void pureRotation_allModulesHaveEqualSpeeds() {
        // simetrik robot geometrisinde saf rotasyonda tüm modüller eşit hız alır
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(
            new ChassisSpeeds(0, 0, Math.PI)
        );
        double speed0 = states[0].speedMetersPerSecond;
        for (SwerveModuleState s : states) {
            assertEquals(speed0, s.speedMetersPerSecond, 1e-6,
                "Simetrik robotun saf rotasyonunda tüm modül hızları eşit olmalı");
        }
    }

    @Test
    void kinematics_fourModulesReturned() {
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(
            new ChassisSpeeds(1.0, 0.5, 0.3)
        );
        assertEquals(4, states.length, "4 modül durumu dönmeli");
    }
}
