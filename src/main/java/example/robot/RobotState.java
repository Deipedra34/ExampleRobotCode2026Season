package example.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Robotun tahmini pozunu merkezi olarak tutan singleton. DriveSubsystem her periodic'te
 * gunceller; VisionSubsystem (Limelight'a SetRobotOrientation gondermek icin) ve ileride
 * pose'a ihtiyac duyan baska her yer (auto, aim-while-move vb.) buradan okur.
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
