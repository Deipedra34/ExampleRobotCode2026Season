package example.robot.subsystems.vision;

import example.robot.lib.LimelightHelpers;
import example.robot.lib.LimelightHelpers.PoseEstimate;

public class VisionIOLimelight implements VisionIO {
    private final String limelightName;

    public VisionIOLimelight(String limelightName) {
        this.limelightName = limelightName;
    }

    @Override
    public void setRobotOrientation(double yawDegrees) {
        LimelightHelpers.SetRobotOrientation(limelightName, yawDegrees, 0, 0, 0, 0, 0);
    }

    @Override
    public void readInputs(VisionIOInputs inputs) {
        inputs.hasTarget = LimelightHelpers.getTV(limelightName);
        inputs.tx = LimelightHelpers.getTX(limelightName);
        inputs.ty = LimelightHelpers.getTY(limelightName);

        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        inputs.mt2Valid = mt2 != null && LimelightHelpers.validPoseEstimate(mt2);
        if (inputs.mt2Valid) {
            inputs.mt2Pose = mt2.pose;
            inputs.mt2TimestampSeconds = mt2.timestampSeconds;
        }

        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        inputs.mt1Valid = mt1 != null;
        if (inputs.mt1Valid) {
            inputs.mt1Pose = mt1.pose;
            inputs.mt1TimestampSeconds = mt1.timestampSeconds;
            inputs.mt1TagCount = mt1.tagCount;
        }
    }
}
