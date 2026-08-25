# Example Robot Code – 2026 Season

This is example/practice FRC robot code for the 2026 season, built around a swerve
drivetrain. It's not the real competition code for any specific team — the team number
in `Constants.java` (`9999`) is a placeholder. Think of it as a reference project for
trying out an architecture, not something pulled straight off a competition robot.

## Architecture

- **AdvantageKit logging** — every subsystem logs through an `IO` interface
  (`readInputs`/`@AutoLog`), so replay and log review work the same in sim and on
  real hardware.
- **IO / Hardware / Sim split per subsystem** — each subsystem (`drive`, `shooter`,
  `intake`, `feeder`, `vision`) has an `XxxIO` interface, an `XxxIOHardware`
  implementation, and an `XxxIOSim` implementation. `RobotContainer` picks which one to
  wire up based on `RobotBase.isReal()`.
- **CTRE generated Swerve API** — the drivetrain uses Tuner X's generated
  `generated/TunerConstants.java` and `subsystems/drive/CommandSwerveDrivetrain.java`
  instead of a hand-rolled `SwerveModule` class.
- **`RobotState`** — a small singleton that holds the current estimated pose, so
  anything that needs it (vision, autos, aim-while-move) can just read it instead of
  passing `DriveSubsystem` around everywhere.
- **`commands/` + `factories/`** — one-off, stateful commands live in `commands/`;
  simple command compositions built from existing subsystem methods live in
  `factories/` so `RobotContainer`'s button bindings stay readable.

## Subsystems

| Subsystem | Notes |
|---|---|
| Drive   | Swerve, CTRE generated API, PathPlanner autos |
| Shooter | 4 flywheels + feeder motor, velocity PID |
| Intake  | Pivot (position PID) + roller, beam-break note sensing |
| Feeder  | Open-loop roller between intake and shooter |
| Vision  | Limelight (MegaTag1 + MegaTag2) fused into the drive pose estimator |

## Building and running

Requires the 2026 WPILib toolchain (GradleRIO handles the rest).

```
./gradlew build          # compile everything
./gradlew test           # unit tests (pure math, no HAL required)
./gradlew simulateJava   # run in WPILib sim
```

## Status

This is a work in progress — several constants (PID gains, gear ratios, setpoints) are
placeholders or guesses that still need to be tuned/verified on real hardware. See the
comments in `Constants.java` for specifics.
