package example.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import example.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterFactory {
    private ShooterFactory() {}

    private static final double SPIN_UP_SECONDS = 1.4;

    /** RB -> manuel: spinup bekle sonra feeder calisir; birakinca durur */
    public static Command manualShoot(ShooterSubsystem shooter) {
        return Commands.sequence(
            Commands.run(shooter::spinUp, shooter).withTimeout(SPIN_UP_SECONDS),
            Commands.run(() -> {
                shooter.spinUp();
                shooter.runFeeder();
            }, shooter)
        ).finallyDo(interrupted -> shooter.stop());
    }
}
