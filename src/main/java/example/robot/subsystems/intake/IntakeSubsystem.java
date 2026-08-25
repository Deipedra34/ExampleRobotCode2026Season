package example.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import example.robot.Constants.IntakeConstants;

import org.littletonrobotics.junction.Logger;

/**
 * B button toggle: stowed at start, deploys on the first press, stows again on the second
 */
public class IntakeSubsystem extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // tracks the B-button toggle state - independent of encoder feedback, just the setpoint we want
    private boolean deployed = false;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.readInputs(inputs);
        Logger.processInputs("Intake", inputs);

        SmartDashboard.putBoolean("Intake/Deployed", isDeployed());
        SmartDashboard.putBoolean("Intake/HasNote", inputs.hasNote);
        // live tuning: edit "Intake/DeployTarget" on SmartDashboard to change this on the fly
        SmartDashboard.putNumber("Intake/DeployTarget",
            SmartDashboard.getNumber("Intake/DeployTarget", IntakeConstants.PIVOT_DEPLOYED_ROT));
    }

    private double getDeployTarget() {
        return SmartDashboard.getNumber("Intake/DeployTarget", IntakeConstants.PIVOT_DEPLOYED_ROT);
    }

    /** drops the intake down to the floor */
    public void deploy() {
        io.setPivotPosition(getDeployTarget());
        deployed = true;
    }

    /** pulls the intake back inside the robot */
    public void stow() {
        io.setPivotPosition(IntakeConstants.PIVOT_STOWED_ROT);
        deployed = false;
    }

    /** B button: deploy if stowed, go back to stow if it's already out */
    public void toggle() {
        if (deployed) {
            stow();
        } else {
            deploy();
        }
    }

    public void stop() {
        stow();
    }

    /** spins the intake roller (x44) while LT is held */
    public void runRoller() {
        io.setRoller(IntakeConstants.ROLLER_INTAKE_SPEED);
    }

    public void stopRoller() {
        io.setRoller(0);
    }

    public boolean hasNote() {
        return inputs.hasNote;
    }

    /** has the pivot reached its target position? */
    public boolean isDeployed() {
        return Math.abs(inputs.pivotPositionRot - getDeployTarget()) < IntakeConstants.PIVOT_TOLERANCE_ROT;
    }

    public boolean isStowed() {
        return Math.abs(inputs.pivotPositionRot - IntakeConstants.PIVOT_STOWED_ROT) < IntakeConstants.PIVOT_TOLERANCE_ROT;
    }
}
