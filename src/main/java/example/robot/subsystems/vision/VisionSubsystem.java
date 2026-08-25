package example.robot.subsystems.vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import example.robot.Constants.VisionConstants;
import example.robot.RobotState;

import org.littletonrobotics.junction.Logger;

/**
 * Limelight'i okuyup gecerli poz olcumlerini DriveSubsystem'e (pose estimator'a) besler.
 * Eskiden DriveSubsystem.fuseVision() icinde olan mantik buraya tasindi.
 */
public class VisionSubsystem extends SubsystemBase {

    @FunctionalInterface
    public interface VisionMeasurementConsumer {
        void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
    }

    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final VisionMeasurementConsumer measurementConsumer;
    private final DoubleSupplier yawRateDegPerSecSupplier;

    public VisionSubsystem(
            VisionIO io,
            VisionMeasurementConsumer measurementConsumer,
            DoubleSupplier yawRateDegPerSecSupplier) {
        this.io = io;
        this.measurementConsumer = measurementConsumer;
        this.yawRateDegPerSecSupplier = yawRateDegPerSecSupplier;
    }

    @Override
    public void periodic() {
        io.setRobotOrientation(RobotState.getInstance().getHeading().getDegrees());
        io.readInputs(inputs);
        Logger.processInputs("Vision", inputs);

        double yawRate = Math.abs(yawRateDegPerSecSupplier.getAsDouble());
        if (yawRate < VisionConstants.MAX_VISION_YAW_RATE_DEG_PER_SEC && inputs.mt2Valid) {
            measurementConsumer.accept(inputs.mt2Pose, inputs.mt2TimestampSeconds, VecBuilder.fill(0.7, 0.7, 9999999));
        }

        if (inputs.mt1Valid && inputs.mt1TagCount >= 2) {
            measurementConsumer.accept(inputs.mt1Pose, inputs.mt1TimestampSeconds, VecBuilder.fill(1.0, 1.0, Math.toRadians(50)));
        }

        SmartDashboard.putBoolean("LL/HasTarget", inputs.hasTarget);
        SmartDashboard.putNumber("LL/tx (deg)", inputs.tx);
        SmartDashboard.putBoolean("LL/Aligned", isAimed());
    }

    /** tx tabanli P kontrol donus duzeltmesi (rad/s); hedef yoksa 0. */
    public double getAimCorrection() {
        if (!inputs.hasTarget) return 0.0;
        return -VisionConstants.AIM_KP * inputs.tx;
    }

    public boolean isAimed() {
        return inputs.hasTarget && Math.abs(inputs.tx) < VisionConstants.AIM_TOLERANCE_DEG;
    }
}
