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

/**
 * SwerveDrivePoseEstimator ın davranışını test eder
 * HAL gerektirmeyen direkt matematik testleri var bu yüzden donanım başlatmaya gerek yok 
 */
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
        assertEquals(0.0, pose.getX(), 1e-6, "Başlangıç X = 0");
        assertEquals(0.0, pose.getY(), 1e-6, "Başlangıç Y = 0");
        assertEquals(0.0, pose.getRotation().getDegrees(), 1e-6, "Başlangıç açı = 0°");
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
        // modüller hareket etmiyorsa (sıfır konum) güncelleme pose u değiştirmemeli
        estimator.update(Rotation2d.fromDegrees(0), zeroPositions);
        Pose2d pose = estimator.getEstimatedPosition();
        assertEquals(0.0, pose.getX(), 1e-6);
        assertEquals(0.0, pose.getY(), 1e-6);
    }

    @Test
    void addVisionMeasurement_pullsPoseTowardVision() {
        // update() WPIUtilJNI monotonic saatini kullanir; sabit 0.02 s bu bufferin
        // cok disinda kalir ve ölçüm reddedilir o yüzden updateWithTime ile deterministik
        // timestamp kullanarak vizyon fusionunu dogru test edebiliriz (estimator ın içindeki zaman tabanlı logic nedeniyle)
        estimator.updateWithTime(1.0, Rotation2d.fromDegrees(0), zeroPositions);

        Pose2d visionPose = new Pose2d(5.0, 0.0, Rotation2d.fromDegrees(0));
        estimator.addVisionMeasurement(visionPose, 1.0);
        estimator.updateWithTime(1.02, Rotation2d.fromDegrees(0), zeroPositions);

        Pose2d result = estimator.getEstimatedPosition();
        // vizyon güveni düşük (std dev 0.5) odometri güveni yüksek (std dev 0.05)
        // bu yüzden pose hafifçe sağa kaymalı ama 5 e ulaşmamalı
        assertTrue(result.getX() > 0.0, "Vizyon ölçümü X'i sıfırdan büyük yapmış olmalı");
        assertTrue(result.getX() < 5.0, "Vizyon tek başına tam 5 m'ye çekmemeli (odometri de etkili)");
    }

    @Test
    void addVisionMeasurement_highConfidence_strongerEffect() {
        SwerveDrivePoseEstimator highConfidence = new SwerveDrivePoseEstimator(
            KINEMATICS,
            Rotation2d.fromDegrees(0),
            zeroPositions,
            new Pose2d(),
            VecBuilder.fill(0.5, 0.5, Math.toRadians(30)),   // odometri güvensiz
            VecBuilder.fill(0.01, 0.01, Math.toRadians(1))   // vizyon çok güvenilir
        );

        highConfidence.updateWithTime(1.0, Rotation2d.fromDegrees(0), zeroPositions);

        Pose2d visionPose = new Pose2d(4.0, 0.0, Rotation2d.fromDegrees(0));
        highConfidence.addVisionMeasurement(visionPose, 1.0);
        highConfidence.updateWithTime(1.02, Rotation2d.fromDegrees(0), zeroPositions);

        double x = highConfidence.getEstimatedPosition().getX();
        assertTrue(x > 2.0, "Yüksek güvenli vizyon ölçümü pose'u önemli ölçüde kaydırmalı");
    }

    @Test
    void multipleUpdates_withMovingModules_advancesPose() {
        // ileri giden modüller (1 metre)
        SwerveModulePosition[] movedPositions = new SwerveModulePosition[] {
            new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
            new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
            new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0)),
            new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0))
        };

        estimator.update(Rotation2d.fromDegrees(0), movedPositions);
        Pose2d pose = estimator.getEstimatedPosition();
        assertTrue(pose.getX() > 0.5, "1 m ileri giden modüller X pose'unu artırmalı");
    }
}
