## FRC 5827 Robot Code (2025) — Student Architecture Guide

This repository contains our command-based robot software from last season.
The goal of this document is to help you, as a new student programmer, understand:

- **How the code is organized**
- **What the major subsystems are**
- **Where to edit things when adding new robot features**
- **How commands, buttons, autonomous, and dashboards fit together**
- **How simulation and logging work with AdvantageKit**

If you get lost, start here and then follow the links to the relevant files.

---

## How the program starts

- **`Robot.java`** (`src/main/java/frc/robot/Robot.java`)
  This is the class WPILib runs automatically. It:
  - Starts AdvantageKit logging
  - Creates a `RobotContainer`
  - Runs the `CommandScheduler` every loop (every 20ms)

  You rarely add robot logic here — it’s mostly framework boilerplate.

- **`RobotContainer.java`** (`src/main/java/frc/robot/RobotContainer.java`)
  **This is the most important file to understand first.**
  It is the “wiring harness” of the code:
  - Creates every subsystem
  - Chooses correct hardware implementations depending on robot mode
  - Sets the default driving command
  - Binds Xbox controller buttons to commands
  - Sets up autonomous selection and PathPlanner named commands

When you add a new mechanism to the robot, **you almost always touch `RobotContainer.java`**.

---

## Simulation vs. real robot vs. different robots (modes)

We support multiple hardware environments using `Constants.currentMode`.

- **`Constants.java`** (`src/main/java/frc/robot/Constants.java`)
  `currentMode` is set at startup:
  - If we are **not on a real roboRIO**, we use `SIM` (or `REPLAY`).
  - If we are on a roboRIO, we read its **serial number** to decide:
    - `COMPETITION` (the full competition bot, “Jacques”)
    - `AGNES` (the smaller swerve-only dev bot)

Why do this? Because **different robots have different hardware installed**, and the code must not
try to control motors/sensors that don’t exist.

### Where the mode matters

In `RobotContainer.java` you’ll see:

- `Mode.COMPETITION`: real TalonFX drive modules, real elevator/end effector, real PhotonVision cameras.
- `Mode.AGNES`: real drive modules, **but “empty” IO objects for mechanisms/cameras that aren’t installed**.
- `Mode.SIM`: simulated drive/modules, simulated elevator/end effector, optional simulated vision.
- `Mode.REPLAY`: disabled IO, but subsystems still run so logs can be replayed.

---

## What is a subsystem?

A **subsystem** represents one control-able part of the robot:
drivetrain, elevator, end effector, climb, intake, vision, etc.

All subsystems live in:

`src/main/java/frc/robot/subsystems/`

Inside that folder, each subsystem has its own directory:

- `drive/` — swerve drivetrain and odometry
- `elevator/` — elevator lift mechanism
- `endEffector/` — coral/algae handling rollers/arms
- `climb/` — climbing mechanism
- `vision/` — camera inputs and pose estimation
- `autos/` — autonomous chooser / PathPlanner integration
- plus other mechanism folders depending on the season

### The AdvantageKit IO pattern (very important)

Most subsystems follow a 3‑layer structure:

1. **`Subsystem.java`**
   High‑level behavior + state machines + commands + logging.
   Example: `elevator/Elevator.java`

2. **`SubsystemIO.java`**
   An interface describing what hardware functions exist.
   Example: `elevator/ElevatorIO.java`

3. **`SubsystemIO<Variant>.java`**
   One or more implementations that talk to **real hardware** or **simulation**.
   Examples:
   - `elevator/ElevatorIOTalonFX.java` (real TalonFX motors)
   - `elevator/ElevatorIOSim.java` (physics simulation)
   - For other subsystems you may see `...IONeo550`, `...IOSparkMax`, etc.

Why split it this way?

- Lets us **simulate** by swapping real IO for sim IO.
- Lets us support **multiple robots** (Jacques vs Agnes).
- Keeps the high‑level code clean and testable.
- Enables automatic logging through AdvantageKit.

When you add a new subsystem, follow this same pattern.

---

## Commands and button bindings

In command‑based programming:

- A **Command** is an action the robot can do (drive, raise elevator, eject coral).
- Subsystems expose commands or methods.
- The `CommandScheduler` runs commands and handles conflicts automatically.

### Where commands live

`src/main/java/frc/robot/commands/`

Example:

- `DriveCommands.java` contains reusable driving commands.

### How buttons run commands

Look at `RobotContainer.configureButtonBindings()`:

- `onTrue(...)`: schedule a command once when the button is pressed.
- `whileTrue(...)`: schedule a command repeatedly while held.
- `onFalse(...)`: run a command when released.

WPILib command-based docs (highly recommended reading):
`https://frcdocs.wpi.edu/en/2024/docs/software/commandbased/index.html` [WPILib Command-Based Programming](https://frcdocs.wpi.edu/en/2024/docs/software/commandbased/index.html)

### Why you see lambdas like `() -> controller.getLeftY()`

Commands often accept **functions** (Suppliers) rather than fixed numbers.
This lets WPILib call those functions every loop to get live joystick values.

Example (from `DriveCommands.joystickDrive`):

- We pass `DoubleSupplier`s into the command.
- The command reads them each iteration.

This is the same “passing functions as parameters” idea described in the WPILib docs.

---

## Autonomous

Autonomous is **not a separate codebase**.
We build autos out of the same commands that run in teleop.

Key pieces:

- **`AutoChooser`** (`subsystems/autos/AutoChooser.java`)
  Publishes a dashboard chooser and returns the selected auto command.

- **PathPlanner + NamedCommands**
  In `RobotContainer.registerNamedCommands()` we register strings like `"Score"` or `"Intake"`.
  PathPlanner auto files can call these by name during a path.

This keeps autos reliable: if a command works in teleop, it should work in auto too.

---

## Vision system (PhotonVision + AprilTags)

We use PhotonVision to detect AprilTags and estimate robot position.
PhotonVision website: `https://photonvision.org/`

### Where the code is

- `subsystems/vision/Vision.java`
  Reads camera observations, filters bad ones, and sends good pose estimates to Drive.

- `subsystems/vision/VisionIOPhotonVision.java`
  Talks to real PhotonVision cameras on the coprocessor.

- `subsystems/vision/VisionIOPhotonVisionSim.java`
  Uses the PhotonVision simulator when running in `SIM` mode.

- `subsystems/vision/VisionConstants.java`
  **Camera names and robot‑to‑camera transforms live here.**

### How cameras are configured

Each camera needs:

1. A **name** matching PhotonVision configuration (example: `"F-STAR"`).
2. A **Transform3d** describing where the camera sits on the robot.

These transforms are used to convert “camera sees tag at X/Y/Z” into “robot is at X/Y on field”.

If transforms are wrong, vision pose estimates will be wrong.

### How vision pose estimates are filtered

In `Vision.periodic()` we reject poses that are likely bad:

- No tags seen
- Single‑tag solve with high ambiguity
- “Robot is flying” (unrealistic Z)
- Pose outside the field
- Rear cameras very far from our current estimated pose

Accepted poses are passed into Drive with measurement noise values so the estimator trusts them
appropriately.

All detections (accepted and rejected) are logged for AdvantageScope visualization.

---

## Driver interface, dashboard, and closest‑zone scoring

Besides the Xbox controller, we use dashboard controls.

### NetworkTables (under the hood)

WPILib dashboards communicate through **NetworkTables**.
LoggedDashboardChooser and LoggedTunableNumber use NetworkTables automatically.

### Where dashboard logic lives
Dashboard-related files and patterns:

- `subsystems/Superstructure.java`
  This subsystem coordinates multiple mechanisms and also owns our scoring dashboard choices:
  - **Reef Level chooser** (L1–L4)
  - **Score Direction chooser** (Left vs Right)
  - **Raise distance tunable**

- `util/LoggedTunableNumber.java`
  Lets a constant be tuned live in “tuning mode” and logged either way.

- `util/LoggedDashboardChooser.java` (from AdvantageKit)
  A SendableChooser wrapper that also logs selections.

### Closest‑zone selection (helping the driver)

In `Superstructure.getNearestZoneIndex()` we:

1. Ask Drive for a **lookahead pose** (where we will be very soon).
2. Compare that pose to reef “zones” in `FieldConstants.Reef.zones`.
3. Select the closest zone index.

When the driver presses an align/score button:

- The robot chooses the nearest zone automatically.
- The driver still chooses **Left vs Right** using the dashboard chooser.

This reduces driver mental load. A simple, consistent control scheme makes matches faster and less
stressful because the driver doesn’t need to think about exact peg numbers under pressure.

---

## Elevator behavior and safety

The elevator is controlled by:

- `subsystems/elevator/Elevator.java` (high‑level behavior)
- `subsystems/elevator/ElevatorIO.java` (hardware interface)
- `subsystems/elevator/ElevatorIOTalonFX.java` (real motors)
- `subsystems/elevator/ElevatorIOSim.java` (simulation)

### How the elevator moves

When a command like `elevatorUp(L4)` runs:

1. `Elevator.closedLoopToPosition(...)` sets a new **goal height**.
2. Each loop, a trapezoid profile generates a smooth next state (position + velocity).
3. Feedforward voltage is computed to “help” the motors move quickly and smoothly.
4. The motor controller runs PID to track the profile.

### Built‑in safety features

We never want the robot to destroy itself. Safety checks include:

- **Coral blocking check**
  The end effector tells Elevator if coral would jam the lift.
  If blocked, the command refuses to move and raises an alert.

- **Soft limits** (in `ElevatorIOTalonFX`)
  Forward and reverse limits stop us from driving beyond real travel.

- **Current limits** (also in `ElevatorIOTalonFX`)
  Prevents motors/wiring damage when stalled.

When you add a new mechanism, think about:

- What physical stops or collisions exist?
- What software limits should prevent them?
- What alerts should warn the driver?

Safety code should be written early — the first deployment is when mistakes break hardware.

---

## Drive subsystem (swerve)

The swerve drive code lives in `subsystems/drive/`.
It handles:

- Swerve module control
- Gyro heading
- Odometry and pose estimation
- Vision measurement fusion

This subsystem is large and mostly template‑based.
You don’t need to memorize all of it right away, but you *do* need to know:

- How to call drive commands (see `DriveCommands.java`)
- Where pose/odometry are logged for AdvantageScope

---

## AdvantageKit, AdvantageScope, and why we log everything

We use AdvantageKit because it gives us:

- **High‑rate structured logging** of every subsystem input/output.
- **Replay mode** — re-run a match from logs to see what the robot *thought* was happening.
- **Simulation support** — view the robot on the field in AdvantageScope while running sim.
- **Dashboard integration** for tunables and choosers.

This workflow helps us debug and improve faster:

1. Run matches / practice.
2. Download logs.
3. Open in AdvantageScope and replay.
4. Find root causes and tune/fix code.

Keeping subsystems split into IO layers is what makes replay and sim accurate.

---

## Where to change constants

Constants are collected in a few places:

- `Constants.java` — CAN IDs, modes, global flags.
- `FieldConstants.java` — field geometry, reef zones, scoring points.
- Each subsystem may have tunables in its own class (example `Elevator/kP`).

**Use constants or tunables instead of hard‑coding numbers in logic.**

---

## Adding a new subsystem (how you will do this this season)

When the game changes, our subsystems change too.
To add a new subsystem:

1. **Create a folder** in `subsystems/yourSubsystem/`.
2. Write:
   - `YourSubsystem.java`
   - `YourSubsystemIO.java`
   - `YourSubsystemIO<Hardware>.java`
   - `YourSubsystemIOSim.java`
3. Add it to `RobotContainer` inside each `currentMode` case.
4. Expose commands in your subsystem.
5. Bind those commands to controller buttons/triggers in `configureButtonBindings()`.

Ask a mentor for a reference subsystem if you’re unsure — Elevator and EndEffector are good models.

---

## Building and running

On Windows PowerShell:

```powershell
$env:JAVA_HOME="C:\Users\Public\wpilib\2025\jdk"
$env:PATH="$env:JAVA_HOME\bin;$env:PATH"
./gradlew.bat build
```

To run simulation, use the WPILib VS Code commands:

- “Simulate Robot Code”
- “Start Driver Station”

---

## Final notes for new programmers

- Start by learning **RobotContainer → Subsystems → Commands**.
- Use the dashboard and tunables to experiment.
- Read logs in AdvantageScope often.
- Keep driver controls simple — the driver’s brain is a limited resource during matches.

We’re excited to have you on the team. Ask questions, and don’t be afraid to experiment.
