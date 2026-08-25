package example.robot.subsystems.intake;

import example.robot.Constants.IntakeConstants;

/**
 * Kinematik seviyede sim: pivot pozisyon setpoint'ini gecikmesiz olarak rotor pozisyonuna yazar.
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
