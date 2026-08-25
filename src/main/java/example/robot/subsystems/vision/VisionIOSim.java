package example.robot.subsystems.vision;

/** there's no real Limelight in sim, so this just stubs it out and never sees a target. */
public class VisionIOSim implements VisionIO {
    @Override
    public void readInputs(VisionIOInputs inputs) {}

    @Override
    public void setRobotOrientation(double yawDegrees) {}
}
