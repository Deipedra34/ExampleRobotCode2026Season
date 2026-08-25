# Contributing

This is an example/practice project, but issues and PRs are welcome if you spot a bug,
have a cleaner way to do something, or want to extend it.

## Getting set up

1. Install the 2026 WPILib toolchain (VS Code + WPILib extension covers everything you need).
2. Clone the repo and open it in VS Code via **WPILib: Open Project**.
3. `./gradlew build` should compile cleanly and `./gradlew test` should pass before you
   start changing things.

## Making changes

- Keep the IO pattern: hardware access belongs in `XxxIOHardware`, sim-only behavior in
  `XxxIOSim`, and subsystem logic (the stuff that doesn't care whether it's real or sim)
  in `XxxSubsystem`. Don't reach into a `TalonFX`/`XboxController`/etc. from outside the
  IO layer.
- New button bindings go through `factories/`, not directly in `RobotContainer`, unless
  the command is a trivial one-liner.
- Run `./gradlew test` before opening a PR. If you're changing something that affects
  simulation, run `./gradlew simulateJava` and check it actually drives correctly.
- Keep constants in `Constants.java`, grouped under the relevant `XxxConstants` class.

## Pull requests

- Keep PRs focused — one change/feature per PR is easier to review than a grab bag.
- Describe *why* the change is needed, not just what changed — that's the part that
  doesn't show up in the diff.
- If you're adding a new constant that still needs tuning, say so in a comment; don't
  leave a bare guessed number with no context.
