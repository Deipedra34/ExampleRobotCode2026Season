package example.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import example.robot.Constants.DriveConstants;
import example.robot.controlboard.ControlBoard;
import example.robot.subsystems.drive.DriveSubsystem;
import example.robot.subsystems.feeder.FeederSubsystem;
import example.robot.subsystems.intake.IntakeSubsystem;
import example.robot.subsystems.shooter.ShooterSubsystem;
import example.robot.subsystems.vision.VisionSubsystem;

/**
 * RT whileTrue:
 *   - driving  -> vision.getAimCorrection() handles rotation, left stick still does X/Y
 *   - shooting -> spin up the flywheels, feeder kicks in once we're at speed
 *   - 3s after the feeder starts -> intake pulses deploy 0.4s / stow 0.4s
 *     (walks whatever notes are still sitting in the intake into the shooter one at a time)
 */
public class AlignShootCommand extends Command {

    private static final double PULSE_HALF_PERIOD_S = 0.4;
    private static final double PULSE_START_DELAY_S = 3.0;

    private final DriveSubsystem   drive;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem  intake;
    private final FeederSubsystem  feeder;
    private final VisionSubsystem  vision;
    private final ControlBoard     controlBoard;

    private final Timer feederTimer = new Timer();
    private final Timer pulseTimer  = new Timer();

    private boolean feederRunning  = false;
    private boolean pulseActive    = false;
    private boolean pulseDeployed  = false;

    public AlignShootCommand(
            DriveSubsystem drive,
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            FeederSubsystem feeder,
            VisionSubsystem vision,
            ControlBoard controlBoard) {
        this.drive        = drive;
        this.shooter       = shooter;
        this.intake        = intake;
        this.feeder        = feeder;
        this.vision         = vision;
        this.controlBoard = controlBoard;
        addRequirements(drive, shooter, intake, feeder);
    }

    @Override
    public void initialize() {
        feederTimer.reset();
        feederTimer.stop();
        pulseTimer.reset();
        pulseTimer.stop();
        feederRunning = false;
        pulseActive   = false;
        pulseDeployed = false;

        intake.stow();
        shooter.spinUp();
    }

    @Override
    public void execute() {
        // --- driving: let vision handle the aim ---
        double xSpeed = controlBoard.getDriveX() * DriveConstants.MAX_SPEED_MPS;
        double ySpeed = controlBoard.getDriveY() * DriveConstants.MAX_SPEED_MPS;
        double rot = vision.getAimCorrection();
        drive.drive(xSpeed, ySpeed, rot, true);

        // --- shooting: spinUp has to be called every loop (Phoenix 6) ---
        shooter.spinUp();
        if (shooter.isAtSpeed()) {
            shooter.runFeeder();
            feeder.runRollers();
            if (!feederRunning) {
                feederRunning = true;
                feederTimer.restart();
            }
        }

        // --- intake pulse, kicks in 3s after the feeder starts ---
        if (feederRunning && feederTimer.hasElapsed(PULSE_START_DELAY_S)) {
            if (!pulseActive) {
                pulseActive   = true;
                pulseDeployed = true;
                intake.deploy();
                pulseTimer.restart();
            } else if (pulseTimer.hasElapsed(PULSE_HALF_PERIOD_S)) {
                pulseTimer.restart();
                pulseDeployed = !pulseDeployed;
                if (pulseDeployed) {
                    intake.deploy();
                } else {
                    intake.stow();
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        intake.stop();
        feeder.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
