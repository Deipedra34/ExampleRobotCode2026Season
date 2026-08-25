package example.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import example.robot.Constants.ShooterConstants;

/**
 * four flywheels (2 left, 2 right) plus a feeder motor
 * flywheels: kraken x60, velocity PID + FF
 * feeder   : kraken x60, duty cycle
 */
public class ShooterIOHardware implements ShooterIO {

    protected final TalonFX leftFlywheel   = new TalonFX(ShooterConstants.LEFT_FLYWHEEL_ID);
    protected final TalonFX leftFlywheel2  = new TalonFX(ShooterConstants.LEFT_FLYWHEEL_2_ID);
    protected final TalonFX rightFlywheel  = new TalonFX(ShooterConstants.RIGHT_FLYWHEEL_ID);
    protected final TalonFX rightFlywheel2 = new TalonFX(ShooterConstants.RIGHT_FLYWHEEL_2_ID);
    protected final TalonFX feeder         = new TalonFX(ShooterConstants.FEEDER_ID);

    private final VelocityVoltage velocityReq = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut      neutralReq  = new NeutralOut();

    public ShooterIOHardware() {
        configFlywheel(leftFlywheel,   ShooterConstants.LEFT_FLYWHEEL_INVERTED);
        configFlywheel(leftFlywheel2,  ShooterConstants.LEFT_FLYWHEEL_2_INVERTED);
        configFlywheel(rightFlywheel,  ShooterConstants.RIGHT_FLYWHEEL_INVERTED);
        configFlywheel(rightFlywheel2, ShooterConstants.RIGHT_FLYWHEEL_2_INVERTED);
        configFeeder();
    }

    private void configFlywheel(TalonFX motor, boolean inverted) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted    = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.Slot0.kP = ShooterConstants.FLYWHEEL_KP;
        cfg.Slot0.kI = ShooterConstants.FLYWHEEL_KI;
        cfg.Slot0.kD = ShooterConstants.FLYWHEEL_KD;
        cfg.Slot0.kS = ShooterConstants.FLYWHEEL_KS;
        cfg.Slot0.kV = ShooterConstants.FLYWHEEL_KV;

        cfg.CurrentLimits.SupplyCurrentLimit       = ShooterConstants.FLYWHEEL_SUPPLY_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit       = ShooterConstants.FLYWHEEL_STATOR_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(cfg);
    }

    private void configFeeder() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted    = ShooterConstants.FEEDER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.CurrentLimits.SupplyCurrentLimit       = ShooterConstants.FEEDER_SUPPLY_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        feeder.getConfigurator().apply(cfg);
    }

    @Override
    public void readInputs(ShooterIOInputs inputs) {
        inputs.leftVelocityRps   = leftFlywheel.getVelocity().getValueAsDouble();
        inputs.left2VelocityRps  = leftFlywheel2.getVelocity().getValueAsDouble();
        inputs.rightVelocityRps  = rightFlywheel.getVelocity().getValueAsDouble();
        inputs.right2VelocityRps = rightFlywheel2.getVelocity().getValueAsDouble();
        inputs.feederDutyCycle   = feeder.getDutyCycle().getValueAsDouble();
    }

    @Override
    public void setVelocity(double targetRps) {
        leftFlywheel.setControl(velocityReq.withVelocity(targetRps));
        leftFlywheel2.setControl(velocityReq.withVelocity(targetRps));
        rightFlywheel.setControl(velocityReq.withVelocity(targetRps));
        rightFlywheel2.setControl(velocityReq.withVelocity(targetRps));
    }

    @Override
    public void setFeeder(double dutyCycle) {
        feeder.set(dutyCycle);
    }

    @Override
    public void stop() {
        leftFlywheel.setControl(neutralReq);
        leftFlywheel2.setControl(neutralReq);
        rightFlywheel.setControl(neutralReq);
        rightFlywheel2.setControl(neutralReq);
        feeder.set(0);
    }
}
