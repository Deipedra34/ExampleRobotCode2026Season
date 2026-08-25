package example.robot.subsystems.drive;

/**
 * CommandSwerveDrivetrain, Utils.isSimulation() ile kendi sim thread'ini (updateSimState) zaten
 * kendi icinde yonetiyor - donanim erisimi ayni sekilde CTRE TalonFX/CANcoder/Pigeon2 sim
 * state'lerinden geciyor. Bu yuzden Hardware'den ayri bir davranis gerekmiyor; IO pattern
 * dosya adlandirmasi tutarli olsun diye ayri sinif olarak birakildi.
 */
public class DriveIOSim extends DriveIOHardware {
}
