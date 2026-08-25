package example.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;

import example.robot.Constants.IntakeConstants;

/**
 * pivot motor (position PID, kraken x60) + intake roller (kraken x44) + beam break sensor
 */
public class IntakeIOHardware implements IntakeIO {

    protected final TalonFX pivot  = new TalonFX(IntakeConstants.PIVOT_ID);
    protected final TalonFX roller = new TalonFX(IntakeConstants.ROLLER_ID);
    protected final DigitalInput beamBreak = new DigitalInput(IntakeConstants.BEAM_BREAK_DIO);

    private final PositionVoltage pivotReq = new PositionVoltage(0).withSlot(0);

    public IntakeIOHardware() {
        configPivot();
        configRoller();
        pivot.setPosition(0);
    }

    private void configPivot() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted    = IntakeConstants.PIVOT_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.Slot0.kP = IntakeConstants.PIVOT_KP;
        cfg.Slot0.kI = IntakeConstants.PIVOT_KI;
        cfg.Slot0.kD = IntakeConstants.PIVOT_KD;

        cfg.Feedback.SensorToMechanismRatio = IntakeConstants.PIVOT_GEAR_RATIO;

        cfg.CurrentLimits.SupplyCurrentLimit       = IntakeConstants.PIVOT_SUPPLY_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = IntakeConstants.PIVOT_STATOR_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        pivot.getConfigurator().apply(cfg);
    }

    private void configRoller() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted    = IntakeConstants.ROLLER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.CurrentLimits.SupplyCurrentLimit       = IntakeConstants.ROLLER_SUPPLY_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = IntakeConstants.ROLLER_STATOR_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        roller.getConfigurator().apply(cfg);
    }

    @Override
    public void readInputs(IntakeIOInputs inputs) {
        inputs.pivotPositionRot       = pivot.getPosition().getValueAsDouble();
        inputs.pivotVelocityRotPerSec = pivot.getVelocity().getValueAsDouble();
        inputs.rollerDutyCycle        = roller.getDutyCycle().getValueAsDouble();
        // beam break is normally-closed -> false means no note
        inputs.hasNote = !beamBreak.get();
    }

    @Override
    public void setPivotPosition(double targetRot) {
        pivot.setControl(pivotReq.withPosition(targetRot));
    }

    @Override
    public void setRoller(double dutyCycle) {
        roller.set(dutyCycle);
    }

    @Override
    public void resetPivotPosition() {
        pivot.setPosition(0);
    }
}
