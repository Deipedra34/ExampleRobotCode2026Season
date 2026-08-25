package example.robot;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

// sanity-checking SwerveDriveKinematics against the numbers in DriveConstants, mostly so
// a bad module offset or a flipped sign somewhere doesn't slip through unnoticed
class SwerveKinematicsTest {

    // copied the robot geometry from Constants.DriveConstants
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
                "driving straight forward, every module should point at 0 rad");
        }
    }

    @Test
    void forwardDrive_allModulesSameSpeed() {
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(
            new ChassisSpeeds(1.5, 0, 0)
        );
        for (SwerveModuleState s : states) {
            assertEquals(1.5, s.speedMetersPerSecond, 1e-6,
                "driving straight, every module should be at the same speed");
        }
    }

    @Test
    void strafeDrive_allModulesPointLeft() {
        // positive ySpeed = left
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(
            new ChassisSpeeds(0, 1.0, 0)
        );
        for (SwerveModuleState s : states) {
            assertEquals(Math.PI / 2, s.angle.getRadians(), 1e-6,
                "pure left strafe, every module should point 90 degrees (left)");
        }
    }

    @Test
    void desaturate_exceedingMaxSpeed_clampsToMax() {
        // something like 10 m/s should get clamped down to MAX_V by desaturate
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(
            new ChassisSpeeds(10.0, 0, 0)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_V);
        for (SwerveModuleState s : states) {
            assertTrue(s.speedMetersPerSecond <= MAX_V + 1e-6,
                "after desaturating, speed shouldn't exceed MAX_SPEED");
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
                "speed under the limit shouldn't be touched");
        }
    }

    @Test
    void pureRotation_allModulesHaveEqualSpeeds() {
        // for a symmetric robot, pure rotation gives every module the same speed
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(
            new ChassisSpeeds(0, 0, Math.PI)
        );
        double speed0 = states[0].speedMetersPerSecond;
        for (SwerveModuleState s : states) {
            assertEquals(speed0, s.speedMetersPerSecond, 1e-6,
                "on a symmetric robot, pure rotation should give equal module speeds");
        }
    }

    @Test
    void kinematics_fourModulesReturned() {
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(
            new ChassisSpeeds(1.0, 0.5, 0.3)
        );
        assertEquals(4, states.length, "should get back 4 module states");
    }
}
