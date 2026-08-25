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

    /** Boot'ta pivot fiziksel olarak stow konumundayken baseline alir. */
    void resetPivotPosition();
}
