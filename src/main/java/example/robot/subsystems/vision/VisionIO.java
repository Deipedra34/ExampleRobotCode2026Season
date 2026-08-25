package example.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean hasTarget = false;
        public double tx = 0.0;
        public double ty = 0.0;

        public boolean mt2Valid = false;
        public Pose2d mt2Pose = new Pose2d();
        public double mt2TimestampSeconds = 0.0;

        public boolean mt1Valid = false;
        public Pose2d mt1Pose = new Pose2d();
        public double mt1TimestampSeconds = 0.0;
        public int mt1TagCount = 0;
    }

    void readInputs(VisionIOInputs inputs);

    /** MegaTag2 icin Limelight'a robot yonelimini (derece) gonderir. */
    void setRobotOrientation(double yawDegrees);
}
