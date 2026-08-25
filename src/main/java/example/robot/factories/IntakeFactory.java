package example.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import example.robot.subsystems.intake.IntakeSubsystem;

public class IntakeFactory {
    private IntakeFactory() {}

    /** B button: deploy if stowed, stow if deployed */
    public static Command toggle(IntakeSubsystem intake) {
        return Commands.runOnce(intake::toggle, intake);
    }

    /** roller spins while LT is held, stops when it's released */
    public static Command runRollerWhileHeld(IntakeSubsystem intake) {
        return Commands.startEnd(intake::runRoller, intake::stopRoller, intake);
    }
}
