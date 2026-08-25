package example.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import example.robot.generated.TunerConstants;

public class DriveIOHardware implements DriveIO {
    protected final CommandSwerveDrivetrain drivetrain;

    public DriveIOHardware() {
        drivetrain = TunerConstants.createDrivetrain();
    }

    @Override
    public void readInputs(DriveIOInputs inputs) {
        SwerveDriveState state = drivetrain.getState();
        inputs.pose = state.Pose;
        inputs.speeds = state.Speeds;
        inputs.moduleStates = state.ModuleStates;
        inputs.moduleTargets = state.ModuleTargets;
        inputs.yawRateDegPerSec = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
    }

    @Override
    public void setControl(SwerveRequest request) {
        drivetrain.setControl(request);
    }

    @Override
    public void zeroHeading() {
        drivetrain.seedFieldCentric();
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        drivetrain.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
    }
}
