package example.robot.subsystems.shooter;

/**
 * kinematic-level sim: writes the setpoint straight into the TalonFX sim states with
 * no lag (not a full FlywheelSim physics model, but good enough for testing command logic).
 */
public class ShooterIOSim extends ShooterIOHardware {

    private double targetRps = 0.0;

    @Override
    public void setVelocity(double targetRps) {
        this.targetRps = targetRps;
        super.setVelocity(targetRps);
    }

    @Override
    public void stop() {
        this.targetRps = 0.0;
        super.stop();
    }

    @Override
    public void readInputs(ShooterIOInputs inputs) {
        leftFlywheel.getSimState().setRotorVelocity(targetRps);
        leftFlywheel2.getSimState().setRotorVelocity(targetRps);
        rightFlywheel.getSimState().setRotorVelocity(targetRps);
        rightFlywheel2.getSimState().setRotorVelocity(targetRps);
        super.readInputs(inputs);
    }
}
