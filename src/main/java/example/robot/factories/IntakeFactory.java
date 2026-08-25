package example.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import example.robot.subsystems.intake.IntakeSubsystem;

public class IntakeFactory {
    private IntakeFactory() {}

    /** B tusu: kapaliysa ac, aciksa stow konumuna don */
    public static Command toggle(IntakeSubsystem intake) {
        return Commands.runOnce(intake::toggle, intake);
    }

    /** LT basili tutulunca alma rulosu doner, birakinca durur */
    public static Command runRollerWhileHeld(IntakeSubsystem intake) {
        return Commands.startEnd(intake::runRoller, intake::stopRoller, intake);
    }
}
