package example.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
    @AutoLog
    class DriveIOInputs {
        public Pose2d pose = new Pose2d();
        public double yawRateDegPerSec = 0.0;
        public ChassisSpeeds speeds = new ChassisSpeeds();
        public SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        public SwerveModuleState[] moduleTargets = new SwerveModuleState[4];
    }

    void readInputs(DriveIOInputs inputs);

    void setControl(SwerveRequest request);

    /** Mevcut yonelimi "ileri" olarak sifirlar (eski gyro.reset() karsiligi). */
    void zeroHeading();

    void resetOdometry(Pose2d pose);

    void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs);
}
