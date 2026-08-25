package example.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

import example.robot.Constants.VisionConstants;
import example.robot.commands.AlignShootCommand;
import example.robot.controlboard.ControlBoard;
import example.robot.factories.AutoFactory;
import example.robot.factories.DriveFactory;
import example.robot.factories.IntakeFactory;
import example.robot.factories.ShooterFactory;
import example.robot.subsystems.drive.DriveIOHardware;
import example.robot.subsystems.drive.DriveIOSim;
import example.robot.subsystems.drive.DriveSubsystem;
import example.robot.subsystems.feeder.FeederIOHardware;
import example.robot.subsystems.feeder.FeederIOSim;
import example.robot.subsystems.feeder.FeederSubsystem;
import example.robot.subsystems.intake.IntakeIOHardware;
import example.robot.subsystems.intake.IntakeIOSim;
import example.robot.subsystems.intake.IntakeSubsystem;
import example.robot.subsystems.shooter.ShooterIOHardware;
import example.robot.subsystems.shooter.ShooterIOSim;
import example.robot.subsystems.shooter.ShooterSubsystem;
import example.robot.subsystems.vision.VisionIOLimelight;
import example.robot.subsystems.vision.VisionIOSim;
import example.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

    private final ControlBoard controlBoard = new ControlBoard();

    private final DriveSubsystem   drive;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem  intake;
    private final FeederSubsystem  feeder;
    private final VisionSubsystem  vision;

    public RobotContainer() {
        if (RobotBase.isReal()) {
            drive   = new DriveSubsystem(new DriveIOHardware());
            shooter = new ShooterSubsystem(new ShooterIOHardware());
            intake  = new IntakeSubsystem(new IntakeIOHardware());
            feeder  = new FeederSubsystem(new FeederIOHardware());
            vision  = new VisionSubsystem(
                new VisionIOLimelight(VisionConstants.LIMELIGHT_NAME),
                drive::addVisionMeasurement,
                drive::getYawRateDegPerSec);
        } else {
            drive   = new DriveSubsystem(new DriveIOSim());
            shooter = new ShooterSubsystem(new ShooterIOSim());
            intake  = new IntakeSubsystem(new IntakeIOSim());
            feeder  = new FeederSubsystem(new FeederIOSim());
            vision  = new VisionSubsystem(
                new VisionIOSim(),
                drive::addVisionMeasurement,
                drive::getYawRateDegPerSec);
        }

        configureDefaultCommands();
        configureBindings();
    }

    private void configureDefaultCommands() {
        drive.setDefaultCommand(DriveFactory.teleopDrive(drive, controlBoard));
    }

    private void configureBindings() {
        // Y -> reset gyro
        controlBoard.zeroHeading().onTrue(DriveFactory.zeroHeading(drive));

        // B -> toggle intake
        controlBoard.intakeToggle().onTrue(IntakeFactory.toggle(intake));

        // LT held -> intake roller spins
        controlBoard.intakeRoller().whileTrue(IntakeFactory.runRollerWhileHeld(intake));

        // RT -> vision-assisted aim + shoot, with the intake pulse loop
        controlBoard.alignAndShoot().whileTrue(
            new AlignShootCommand(drive, shooter, intake, feeder, vision, controlBoard));

        // RB -> manual shot
        controlBoard.manualShoot().whileTrue(ShooterFactory.manualShoot(shooter));
    }

    public Command getAutonomousCommand() {
        return AutoFactory.autonomousCommand();
    }
}
