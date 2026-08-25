package example.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import example.robot.Constants.FeederConstants;

import org.littletonrobotics.junction.Logger;

public class FeederSubsystem extends SubsystemBase {

    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    public FeederSubsystem(FeederIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.readInputs(inputs);
        Logger.processInputs("Feeder", inputs);
    }

    public void runRollers() {
        io.setRoller(FeederConstants.ROLLER_INTAKE_SPEED);
    }

    public void reverseRollers() {
        io.setRoller(FeederConstants.ROLLER_OUTTAKE_SPEED);
    }

    public void stopRollers() {
        io.setRoller(0);
    }
}
