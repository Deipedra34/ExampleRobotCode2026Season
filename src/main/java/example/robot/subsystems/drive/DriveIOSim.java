package example.robot.subsystems.drive;

// CTRE already handles the sim thread for us (CommandSwerveDrivetrain kicks it off via
// updateSimState when Utils.isSimulation() is true), and it goes through the same
// TalonFX/CANcoder/Pigeon2 sim states regardless. this subclass is basically just here so
// DriveIOHardware/DriveIOSim matches the pattern the other subsystems use
public class DriveIOSim extends DriveIOHardware {
}
