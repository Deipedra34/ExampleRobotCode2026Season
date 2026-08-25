package example.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import example.robot.Constants.IntakeConstants;

import org.littletonrobotics.junction.Logger;

/**
 * b tusu toggle: basta stow, ilk basista deploy, ikinci basista tekrar stow
 */
public class IntakeSubsystem extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // b tusu toggle durumu - encoder feedback'ten bagimsiz, hedeflenen konumu tutar
    private boolean deployed = false;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.readInputs(inputs);
        Logger.processInputs("Intake", inputs);

        SmartDashboard.putBoolean("Intake/Acildi",   isDeployed());
        SmartDashboard.putBoolean("Intake/Nota Var", inputs.hasNote);
        // canli tune: SmartDashboard'dan "Intake/Deploy Hedef" yazarak degistiriyo
        SmartDashboard.putNumber("Intake/Deploy Hedef",
            SmartDashboard.getNumber("Intake/Deploy Hedef", IntakeConstants.PIVOT_DEPLOYED_ROT));
    }

    private double getDeployTarget() {
        return SmartDashboard.getNumber("Intake/Deploy Hedef", IntakeConstants.PIVOT_DEPLOYED_ROT);
    }

    /** intakei zemine indir */
    public void deploy() {
        io.setPivotPosition(getDeployTarget());
        deployed = true;
    }

    /** intakei robot icine topla */
    public void stow() {
        io.setPivotPosition(IntakeConstants.PIVOT_STOWED_ROT);
        deployed = false;
    }

    /** b tusu: kapaliysa ac, aciksa eski (stow) konumuna dondur */
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

    /** lt basili tutulunca alma rulosunu (x44) dondur */
    public void runRoller() {
        io.setRoller(IntakeConstants.ROLLER_INTAKE_SPEED);
    }

    public void stopRoller() {
        io.setRoller(0);
    }

    public boolean hasNote() {
        return inputs.hasNote;
    }

    /** pivot hedef konuma ulasti mi? */
    public boolean isDeployed() {
        return Math.abs(inputs.pivotPositionRot - getDeployTarget()) < IntakeConstants.PIVOT_TOLERANCE_ROT;
    }

    public boolean isStowed() {
        return Math.abs(inputs.pivotPositionRot - IntakeConstants.PIVOT_STOWED_ROT) < IntakeConstants.PIVOT_TOLERANCE_ROT;
    }
}
