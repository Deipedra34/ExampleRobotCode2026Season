package example.robot.subsystems.shooter;

/**
 * Kinematik seviyede sim: setpoint'i gecikmesiz olarak TalonFX sim state'lerine yazar
 * (tam FlywheelSim fizik modeli degil - komut/mantik testleri icin yeterli).
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
