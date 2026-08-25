package example.robot;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;

/** wanted to make sure I actually understood how the vision std-devs trade off against
 * odometry before wiring VisionSubsystem up for real - this is just exercising that. */
class PoseEstimatorTest {

    private static final double TRACK = Units.inchesToMeters(23.5);
    private static final double BASE  = Units.inchesToMeters(23.5);

    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d( BASE / 2,  TRACK / 2),
        new Translation2d( BASE / 2, -TRACK / 2),
        new Translation2d(-BASE / 2,  TRACK / 2),
        new Translation2d(-BASE / 2, -TRACK / 2)
    );

    private SwerveDrivePoseEstimator estimator;
    private SwerveModulePosition[] zeroPositions;

    @BeforeEach
    void setUp() {
        zeroPositions = new SwerveModulePosition[] {
            new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
            new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
            new SwerveModulePosition(0, Rotation2d.fromDegrees(0)),
            new SwerveModulePosition(0, Rotation2d.fromDegrees(0))
        };

        estimator = new SwerveDrivePoseEstimator(
            KINEMATICS,
            Rotation2d.fromDegrees(0),
            zeroPositions,
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Math.toRadians(5)),
            VecBuilder.fill(0.5, 0.5, Math.toRadians(30))
        );
    }

    @Test
    void initialPose_isOrigin() {
        Pose2d pose = estimator.getEstimatedPosition();
        assertEquals(0.0, pose.getX(), 1e-6, "starting X should be 0");
        assertEquals(0.0, pose.getY(), 1e-6, "starting Y should be 0");
        assertEquals(0.0, pose.getRotation().getDegrees(), 1e-6, "starting angle should be 0 deg");
    }

    @Test
    void resetPosition_updatesImmediately() {
        Pose2d newPose = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(45));
        estimator.resetPosition(Rotation2d.fromDegrees(45), zeroPositions, newPose);

        Pose2d result = estimator.getEstimatedPosition();
        assertEquals(3.0, result.getX(), 1e-6);
        assertEquals(2.0, result.getY(), 1e-6);
        assertEquals(45.0, result.getRotation().getDegrees(), 1e-6);
    }

    @Test
    void updateWithStaticModules_poseDoesNotDrift() {
        // if the modules haven't moved (zero position), updating shouldn't change the pose
        estimator.update(Rotation2d.fromDegrees(0), zeroPositions);
        Pose2d pose = estimator.getEstimatedPosition();
        assertEquals(0.0, pose.getX(), 1e-6);
        assertEquals(0.0, pose.getY(), 1e-6);
    }

    @Test
    void addVisionMeasurement_pullsPoseTowardVision() {
        // update() uses WPIUtilJNI's monotonic clock; a fixed 0.02s timestamp falls way
        // outside that buffer and gets rejected, so we use updateWithTime with a deterministic
        // timestamp instead - that lets us actually test the vision fusion given how the
        // estimator's internal time-based logic works
        estimator.updateWithTime(1.0, Rotation2d.fromDegrees(0), zeroPositions);

        Pose2d visionPose = new Pose2d(5.0, 0.0, Rotation2d.fromDegrees(0));
        estimator.addVisionMeasurement(visionPose, 1.0);
        estimator.updateWithTime(1.02, Rotation2d.fromDegrees(0), zeroPositions);

        Pose2d result = estimator.getEstimatedPosition();
        // vision trust is low here (std dev 0.5), odometry trust is high (std dev 0.05),
        // so the pose should drift slightly toward it but not snap all the way to 5
        assertTrue(result.getX() > 0.0, "the vision measurement should have pulled X above zero");
        assertTrue(result.getX() < 5.0, "vision alone shouldn't pull it all the way to 5 (odometry still counts)");
    }

    @Test
    void addVisionMeasurement_highConfidence_strongerEffect() {
        SwerveDrivePoseEstimator highConfidence = new SwerveDrivePoseEstimator(
            KINEMATICS,
            Rotation2d.fromDegrees(0),
            zeroPositions,
            new Pose2d(),
            VecBuilder.fill(0.5, 0.5, Math.toRadians(30)),   // odometry not trusted much
            VecBuilder.fill(0.01, 0.01, Math.toRadians(1))   // vision trusted a lot
        );

        highConfidence.updateWithTime(1.0, Rotation2d.fromDegrees(0), zeroPositions);

        Pose2d visionPose = new Pose2d(4.0, 0.0, Rotation2d.fromDegrees(0));
        highConfidence.addVisionMeasurement(visionPose, 1.0);
        highConfidence.updateWithTime(1.02, Rotation2d.fromDegrees(0), zeroPositions);

        double x = highConfidence.getEstimatedPosition().getX();
        assertTrue(x > 2.0, "a high-confidence vision measurement should pull the pose a lot");
    }

    @Test
    void multipleUpdates_withMovingModules_advancesPose() {
        // modules that drove forward (1 meter)
        SwerveModulePosition[] movedPositions = new SwerveModulePosition[] {
            new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
            new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
            new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
            new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0))
        };

        estimator.update(Rotation2d.fromDegrees(0), movedPositions);
        Pose2d pose = estimator.getEstimatedPosition();
        assertTrue(pose.getX() > 0.5, "modules that moved 1m forward should push the X pose up");
    }
}
