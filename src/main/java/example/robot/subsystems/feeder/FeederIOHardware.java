package example.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import example.robot.Constants.FeederConstants;

public class FeederIOHardware implements FeederIO {

    protected final TalonFX roller = new TalonFX(FeederConstants.ROLLER_ID);

    public FeederIOHardware() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = FeederConstants.ROLLER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.CurrentLimits.SupplyCurrentLimit       = FeederConstants.ROLLER_SUPPLY_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = FeederConstants.ROLLER_STATOR_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        roller.getConfigurator().apply(cfg);
    }

    @Override
    public void readInputs(FeederIOInputs inputs) {
        inputs.rollerVelocityRps = roller.getVelocity().getValueAsDouble();
        inputs.rollerDutyCycle   = roller.getDutyCycle().getValueAsDouble();
    }

    @Override
    public void setRoller(double dutyCycle) {
        roller.set(dutyCycle);
    }
}
