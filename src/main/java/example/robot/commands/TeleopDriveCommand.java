package example.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import example.robot.Constants.DriveConstants;
import example.robot.controlboard.ControlBoard;
import example.robot.subsystems.drive.DriveSubsystem;

/** teleop surus komutu - field-relative (varsayilan ve tek mod) */
public class TeleopDriveCommand extends Command {

    private final DriveSubsystem drive;
    private final ControlBoard controlBoard;

    public TeleopDriveCommand(DriveSubsystem drive, ControlBoard controlBoard) {
        this.drive = drive;
        this.controlBoard = controlBoard;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drive(
            controlBoard.getDriveX() * DriveConstants.MAX_SPEED_MPS,
            controlBoard.getDriveY() * DriveConstants.MAX_SPEED_MPS,
            controlBoard.getDriveRotation() * DriveConstants.MAX_ANGULAR_RPS,
            true
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
