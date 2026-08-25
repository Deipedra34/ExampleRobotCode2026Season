package example.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Keeps the robot's estimated pose in one place instead of scattering it around.
 * DriveSubsystem updates it every periodic; VisionSubsystem reads it to send
 * SetRobotOrientation to the Limelight, and anything else that ends up needing pose
 * later (auto, aim-while-move, whatever) can just pull it from here.
 */
public class RobotState {
    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private Pose2d estimatedPose = new Pose2d();

    private RobotState() {}

    public void setEstimatedPose(Pose2d pose) {
        estimatedPose = pose;
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public Rotation2d getHeading() {
        return estimatedPose.getRotation();
    }
}
