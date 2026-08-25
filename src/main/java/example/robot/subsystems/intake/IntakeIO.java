package example.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double pivotPositionRot = 0.0;
        public double pivotVelocityRotPerSec = 0.0;
        public double rollerDutyCycle = 0.0;
        public boolean hasNote = false;
    }

    void readInputs(IntakeIOInputs inputs);

    void setPivotPosition(double targetRot);

    void setRoller(double dutyCycle);

    /** call this while the pivot is physically stowed at boot, to set the baseline. */
    void resetPivotPosition();
}
