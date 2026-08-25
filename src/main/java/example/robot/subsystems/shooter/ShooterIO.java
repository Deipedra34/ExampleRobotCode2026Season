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

    /** spins all four flywheels up to the given target speed (RPS). */
    void setVelocity(double targetRps);

    void setFeeder(double dutyCycle);

    /** stops flywheels + feeder (coast). */
    void stop();
}
