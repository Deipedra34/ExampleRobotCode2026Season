package example.robot.subsystems.intake;

import example.robot.Constants.IntakeConstants;

/**
 * kinematic-level sim: writes the pivot's position setpoint straight into the rotor
 * position with no lag.
 */
public class IntakeIOSim extends IntakeIOHardware {

    private double targetPositionRot = 0.0;

    @Override
    public void setPivotPosition(double targetRot) {
        this.targetPositionRot = targetRot;
        super.setPivotPosition(targetRot);
    }

    @Override
    public void resetPivotPosition() {
        this.targetPositionRot = 0.0;
        super.resetPivotPosition();
    }

    @Override
    public void readInputs(IntakeIOInputs inputs) {
        pivot.getSimState().setRawRotorPosition(targetPositionRot * IntakeConstants.PIVOT_GEAR_RATIO);
        pivot.getSimState().setRotorVelocity(0);
        super.readInputs(inputs);
    }
}
