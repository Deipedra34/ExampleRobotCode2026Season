package example.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public double leftVelocityRps = 0.0;
        public double left2VelocityRps = 0.0;
        public double rightVelocityRps = 0.0;
        public double right2VelocityRps = 0.0;
        public double feederDutyCycle = 0.0;
    }

    void readInputs(ShooterIOInputs inputs);

    /** Dort flywheeli de verilen hedef hiza (RPS) cikarir. */
    void setVelocity(double targetRps);

    void setFeeder(double dutyCycle);

    /** Flywheel + feeder durdurur (coast). */
    void stop();
}
