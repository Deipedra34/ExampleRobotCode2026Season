package example.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    class FeederIOInputs {
        public double rollerVelocityRps = 0.0;
        public double rollerDutyCycle = 0.0;
    }

    void readInputs(FeederIOInputs inputs);

    void setRoller(double dutyCycle);
}
