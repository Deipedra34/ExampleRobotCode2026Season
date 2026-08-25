package example.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import example.robot.RobotState;

import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {

    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

    public DriveSubsystem(DriveIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.readInputs(inputs);
        Logger.processInputs("Drive", inputs);
        RobotState.getInstance().setEstimatedPose(inputs.pose);

        SmartDashboard.putNumber("Gyro (deg)", inputs.pose.getRotation().getDegrees());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (fieldRelative) {
            io.setControl(fieldCentricRequest.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rot));
        } else {
            io.setControl(robotCentricRequest.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rot));
        }
    }

    public void stopModules() {
        io.setControl(brakeRequest);
    }

    /** zeros out current heading as "forward" (same as the old Y button behavior). */
    public void zeroHeading() {
        io.zeroHeading();
    }

    public Pose2d getPose() {
        return inputs.pose;
    }

    public double getYawRateDegPerSec() {
        return inputs.yawRateDegPerSec;
    }

    public void resetPose(Pose2d pose) {
        io.resetOdometry(pose);
    }

    /** called by VisionSubsystem whenever it has a valid camera measurement. */
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        io.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
    }
}
