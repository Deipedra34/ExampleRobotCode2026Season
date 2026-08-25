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

        SmartDashboard.putNumber("Shooter/TargetRPS", ShooterConstants.SHOOT_RPS);
        SmartDashboard.putBoolean("Shooter/Ready", isAtSpeed());
    }

    /** spins all four flywheels up to the target speed */
    public void spinUp() {
        io.setVelocity(ShooterConstants.SHOOT_RPS);
    }

    /** spins up to a custom speed (for different distances/angles) */
    public void spinUp(double targetRPS) {
        io.setVelocity(targetRPS);
    }

    /** pushes the note into the flywheels */
    public void runFeeder() {
        io.setFeeder(ShooterConstants.FEEDER_SPEED);
    }

    /** runs the feeder backwards to clear a jam */
    public void reverseFeeder() {
        io.setFeeder(ShooterConstants.FEEDER_REVERSE_SPEED);
    }

    public void stopFeeder() {
        io.setFeeder(0);
    }

    /** stops flywheels + feeder */
    public void stop() {
        io.stop();
    }

    /** are all four flywheels within tolerance of target speed? */
    public boolean isAtSpeed() {
        double tol = ShooterConstants.SPEED_TOLERANCE_RPS;
        double target = ShooterConstants.SHOOT_RPS;
        return Math.abs(inputs.leftVelocityRps   - target) < tol
            && Math.abs(inputs.left2VelocityRps  - target) < tol
            && Math.abs(inputs.rightVelocityRps  - target) < tol
            && Math.abs(inputs.right2VelocityRps - target) < tol;
    }
}
