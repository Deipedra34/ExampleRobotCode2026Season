package example.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import example.robot.controlboard.ControlBoard;
import example.robot.subsystems.drive.DriveSubsystem;

public class DriveFactory {
    private DriveFactory() {}

    public static Command zeroHeading(DriveSubsystem drive) {
        return Commands.runOnce(drive::zeroHeading, drive);
    }

    public static Command teleopDrive(DriveSubsystem drive, ControlBoard controlBoard) {
        return new example.robot.commands.TeleopDriveCommand(drive, controlBoard);
    }
}
