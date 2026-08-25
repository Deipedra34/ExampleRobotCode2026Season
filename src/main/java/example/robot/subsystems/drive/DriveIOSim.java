package example.robot.subsystems.drive;

/**
 * CommandSwerveDrivetrain already runs its own sim thread (updateSimState) whenever
 * Utils.isSimulation() is true, and device access goes through the same CTRE
 * TalonFX/CANcoder/Pigeon2 sim states either way. So there's nothing to do differently
 * from Hardware here - this class only exists to keep the IO naming pattern consistent.
 */
public class DriveIOSim extends DriveIOHardware {
}
