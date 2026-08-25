package example.robot.factories;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoFactory {
    private AutoFactory() {}

    private static final String DEFAULT_AUTO_NAME = "New Auto";

    /** deploy/pathplanner/autos/New Auto.auto dosyasini calistirir; bulunamazsa bos komut doner. */
    public static Command autonomousCommand() {
        try {
            return new PathPlannerAuto(DEFAULT_AUTO_NAME);
        } catch (Exception e) {
            return Commands.none();
        }
    }
}
