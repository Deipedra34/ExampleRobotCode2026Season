package example.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import example.robot.Constants.ShooterConstants;

import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.readInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        SmartDashboard.putNumber("Shooter/Hedef RPS", ShooterConstants.SHOOT_RPS);
        SmartDashboard.putBoolean("Shooter/Hazir", isAtSpeed());
    }

    /** dort flywheeli hedef hiza cikar */
    public void spinUp() {
        io.setVelocity(ShooterConstants.SHOOT_RPS);
    }

    /** ozel hizda spinup (farkli mesafe/aci icin) */
    public void spinUp(double targetRPS) {
        io.setVelocity(targetRPS);
    }

    /** notu flywheele dogru iter */
    public void runFeeder() {
        io.setFeeder(ShooterConstants.FEEDER_SPEED);
    }

    /** feederi geri cevir (sikisma acma) */
    public void reverseFeeder() {
        io.setFeeder(ShooterConstants.FEEDER_REVERSE_SPEED);
    }

    public void stopFeeder() {
        io.setFeeder(0);
    }

    /** flywheel + feeder durdur */
    public void stop() {
        io.stop();
    }

    /** dort flywheel de hedef hiz bandinda mi? */
    public boolean isAtSpeed() {
        double tol = ShooterConstants.SPEED_TOLERANCE_RPS;
        double target = ShooterConstants.SHOOT_RPS;
        return Math.abs(inputs.leftVelocityRps   - target) < tol
            && Math.abs(inputs.left2VelocityRps  - target) < tol
            && Math.abs(inputs.rightVelocityRps  - target) < tol
            && Math.abs(inputs.right2VelocityRps - target) < tol;
    }
}
