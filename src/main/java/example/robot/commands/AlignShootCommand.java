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
 *   - surus  -> vision.getAimCorrection() ile otomatik rotasyon, sol cubuk driver kontrolunde
 *   - atis   -> flywheel spinup, hiza gelince feeder devreye girer
 *   - 3 sn atis sonrasi -> intake 0.4 s deploy + 0.4 s stow dongusu
 *     (intake bolgesindeki kalan notalar birer birer shooter'a beslenir)
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
        // --- surus: vision hizalama ---
        double xSpeed = controlBoard.getDriveX() * DriveConstants.MAX_SPEED_MPS;
        double ySpeed = controlBoard.getDriveY() * DriveConstants.MAX_SPEED_MPS;
        double rot = vision.getAimCorrection();
        drive.drive(xSpeed, ySpeed, rot, true);

        // --- atis: spinup her dongude cagrilmali (Phoenix 6) ---
        shooter.spinUp();
        if (shooter.isAtSpeed()) {
            shooter.runFeeder();
            feeder.runRollers();
            if (!feederRunning) {
                feederRunning = true;
                feederTimer.restart();
            }
        }

        // --- intake pulse: feeder 3 sn calistiktan sonra ---
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
