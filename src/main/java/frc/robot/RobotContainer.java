package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.FieldConstants.ReefLevel;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.autos.AutoChooser;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.endEffector.*;
import frc.robot.subsystems.vision.*;

/**
 * RobotContainer is the "wiring harness" of our software.
 *
 * <p>If you are new to command-based programming, think of this class as the place where we:
 *
 * <ul>
 *   <li>Create each {@code Subsystem} (drive, elevator, end effector, etc.). A subsystem represents
 *       one physical part of the robot that we control.
 *   <li>Create or reference {@code Command}s that tell subsystems what to do.
 *   <li>Bind controller buttons/triggers to those commands.
 *   <li>Set up autonomous mode by choosing commands to run automatically.
 * </ul>
 *
 * <p>Because command-based is a declarative framework, the {@link Robot} class mostly just runs the
 * scheduler each loop. All the interesting setup happens here.
 */
public class RobotContainer {
    // Subsystems
    private final EndEffector endEffector;
    private final Drive drive;
    private final Vision vision;
    private final Climb climb;
    private final Elevator elevator;

    private final AutoChooser autoChooser;
    private Command auto;

    private final Superstructure superstructure;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    /**
     * Construct the robot container.
     *
     * <p>IMPORTANT: The first thing we do is look at {@link Constants#currentMode}. This tells us
     * which hardware is actually on the robot right now (competition bot vs. Agnes swerve bot vs.
     * simulator). We then create each subsystem using the correct IO implementation for that mode.
     *
     * <p>This pattern is used by AdvantageKit: each subsystem is split into:
     *
     * <ul>
     *   <li>{@code Subsystem} class (high-level behavior, commands, logging)
     *   <li>{@code SubsystemIO} interface (what hardware functions exist)
     *   <li>{@code SubsystemIOReal / SubsystemIOTalonFX / SubsystemIOSim} implementations (how to
     *       talk to real motors/sensors or a simulator)
     * </ul>
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case COMPETITION:
                // COMPETITION MODE:
                // We are running on the full competition robot ("Jacques").
                // Instantiate IO classes that talk to real hardware.
                drive =
                        new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight));

                // Vision uses PhotonVision cameras and AprilTags to estimate robot pose.
                // We pass method references into Vision so it can:
                //  - call drive.addVisionMeasurement(...) when it has a new pose estimate
                //  - read the current drive pose for filtering checks
                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                drive::getPose,
                                new VisionIOPhotonVision(
                                        VisionConstants.camera0Name,
                                        VisionConstants.robotToCamera0),
                                new VisionIOPhotonVision(
                                        VisionConstants.camera1Name,
                                        VisionConstants.robotToCamera1),
                                new VisionIOPhotonVision(
                                        VisionConstants.camera2Name,
                                        VisionConstants.robotToCamera2),
                                new VisionIOPhotonVision(
                                        VisionConstants.camera3Name,
                                        VisionConstants.robotToCamera3));

                // Climb subsystem (currently enabled, but you can disable by swapping IO to empty).
                climb = new Climb(new ClimbIOTalonFX() {});

                endEffector = new EndEffector(new EndEffectorIONeo550());

                elevator =
                        new Elevator(new ElevatorIOTalonFX(), endEffector::isCoralBlockingElevator);
                break;

            case AGNES:
                // AGNES MODE:
                // This is our smaller/dev swerve-only robot ("Agnes").
                // It has drive hardware but not the full set of mechanisms or vision.
                drive =
                        new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight));

                // No physical cameras on Agnes, so VisionIO is left empty.
                // The Vision subsystem still exists so the rest of the code doesn't need special
                // cases.
                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                drive::getPose,
                                new VisionIO() {},
                                new VisionIO() {},
                                new VisionIO() {},
                                new VisionIO() {});

                climb = new Climb(new ClimbIOTalonFX());

                // No end effector on Agnes.
                endEffector = new EndEffector(new EndEffectorIO() {});

                // No elevator hardware on Agnes.
                elevator = new Elevator(new ElevatorIO() {}, endEffector::isCoralBlockingElevator);
                break;

            case SIM:
                // SIM MODE:
                // We are running in simulation on a computer.
                // Instantiate IO classes that simulate physics instead of talking to hardware.
                drive =
                        new Drive(
                                new GyroIO() {},
                                new ModuleIOSim(TunerConstants.FrontLeft),
                                new ModuleIOSim(TunerConstants.FrontRight),
                                new ModuleIOSim(TunerConstants.BackLeft),
                                new ModuleIOSim(TunerConstants.BackRight));
                // We can optionally simulate vision. This is useful for testing auto/pose logic.
                if (Constants.simWithVision) {
                    vision =
                            new Vision(
                                    drive::addVisionMeasurement,
                                    drive::getPose,
                                    new VisionIOPhotonVisionSim(
                                            VisionConstants.camera0Name,
                                            VisionConstants.robotToCamera0,
                                            drive::getPose),
                                    new VisionIOPhotonVisionSim(
                                            VisionConstants.camera1Name,
                                            VisionConstants.robotToCamera1,
                                            drive::getPose),
                                    new VisionIOPhotonVisionSim(
                                            VisionConstants.camera2Name,
                                            VisionConstants.robotToCamera2,
                                            drive::getPose),
                                    new VisionIOPhotonVisionSim(
                                            VisionConstants.camera3Name,
                                            VisionConstants.robotToCamera3,
                                            drive::getPose));
                } else {
                    // Vision disabled in sim, so use empty IO.
                    vision =
                            new Vision(
                                    drive::addVisionMeasurement,
                                    drive::getPose,
                                    new VisionIO() {},
                                    new VisionIO() {},
                                    new VisionIO() {},
                                    new VisionIO() {});
                }

                climb = new Climb(new ClimbIOSim());

                endEffector = new EndEffector(new EndEffectorIOSim());

                elevator = new Elevator(new ElevatorIOSim(), endEffector::isCoralBlockingElevator);
                break;

            default:
                // REPLAY MODE (or any unknown mode):
                // We are replaying a log file with AdvantageKit.
                // Hardware is disabled, but subsystems still run so we can analyze behavior.
                drive =
                        new Drive(
                                new GyroIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {});

                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                drive::getPose,
                                new VisionIO() {},
                                new VisionIO() {},
                                new VisionIO() {},
                                new VisionIO() {});

                climb = new Climb(new ClimbIO() {});

                endEffector = new EndEffector(new EndEffectorIO() {});

                elevator = new Elevator(new ElevatorIO() {}, endEffector::isCoralBlockingElevator);
                break;
        }

        // AutoChooser builds a dashboard drop-down to select autos.
        // The boolean enables/disables PathPlanner autos vs SysId testing.
        this.autoChooser = new AutoChooser(drive, true);

        autoChooser.onChange(this::updateAutonomousCommand);

        // Superstructure is a helper subsystem that coordinates multiple mechanisms together
        // (ex: align to a reef zone while raising elevator and preparing end effector).
        superstructure = new Superstructure(drive, endEffector, elevator);

        // NamedCommands allow PathPlanner to reference our commands by string name in auto paths.
        registerNamedCommands();

        // Simple example autonomous for testing (drive forward then stop).
        // This is just a safe default so the robot will do *something* if no auto is selected.
        // In real competition, the dashboard `AutoChooser` overrides this:
        //  - `autoChooser.onChange(this::updateAutonomousCommand)` listens for selection changes
        //  - `updateAutonomousCommand()` (below) replaces `auto` with the chosen routine
        auto =
                Commands.sequence(
                        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(1, 0, 0)), drive)
                                .alongWith(Commands.print("Driving forward!"))
                                .withTimeout(3),
                        Commands.runOnce(() -> drive.stop(), drive)
                                .alongWith(Commands.print("Stopping!")));

        // Configure the button bindings (controller -> commands)
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // DEFAULT COMMAND:
        // While no other drive command is scheduled, `joystickDrive(...)` runs continuously.
        // The lambda suppliers (() -> ...) read joystick values each loop.
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));

        // Button bindings:
        // `onTrue` runs once when pressed, `whileTrue` runs continuously while held, and
        // `onFalse` runs once when released. This is the command-based way to map buttons to robot
        // actions.
        controller.povLeft().onTrue(endEffector.algaeRemoverOpen());
        controller.povRight().onTrue(endEffector.algaeRemoverClose());
        controller.y().onTrue(climb.putAwayIntake()).onTrue(climb.winchArmDeploy());
        controller.povUp().whileTrue(climb.winchArmOut());
        controller.povDown().whileTrue(climb.winchArmIn());
        controller.rightTrigger().onTrue(superstructure.scoreCommand());
        // controller.back().whileTrue(climb.winchArmDeploy());
        // controller.start().whileTrue(climb.winchArmClimb());
        controller
                .leftTrigger()
                .onTrue(endEffector.algaeRemoverClose())
                .onTrue(climb.takeOutIntake())
                .whileTrue(superstructure.intakeCommand());

        controller
                .rightBumper()
                .whileTrue(superstructure.manualElevatorUp())
                .onFalse(elevator.elevatorDown());
        controller
                .a()
                .onTrue(endEffector.algaeRemoverClose())
                .whileTrue(superstructure.alignToScore())
                .whileTrue(superstructure.elevatorScorePosition())
                .onFalse(elevator.elevatorDown());

        controller.x().whileTrue(superstructure.dealgifyCommand()).onFalse(elevator.elevatorDown());
        // controller.povUp().whileTrue(elevator.openLoopElevatorUp());
        // controller.povDown().whileTrue(elevator.openLoopElevatorDown());
    }

    private void registerNamedCommands() {
        // These commands can be called from PathPlanner by name.
        // Notice we reuse the same commands as teleop; autos should be built from tested commands.
        NamedCommands.registerCommand(
                "Score",
                Commands.sequence(
                                elevator.elevatorUp(ReefLevel.L4),
                                Commands.waitUntil(elevator::isAtL4), // in case elevator is blocked
                                endEffector.drawback(),
                                endEffector.ejectToReef().withTimeout(2.0))
                        .alongWith(Commands.runOnce(drive::stop))
                        .alongWith(Commands.print("Scoring!")));
        NamedCommands.registerCommand(
                "Intake",
                Commands.sequence(elevator.goToStationHeight(), endEffector.takeFromIntake())
                        .alongWith(Commands.runOnce(drive::stop))
                        .alongWith(Commands.print("Intaking!")));
        NamedCommands.registerCommand(
                "Prepare to Score",
                elevator.elevatorUp(ReefLevel.L4).alongWith(Commands.print("Raising elevator!")));
        NamedCommands.registerCommand(
                "Prepare to Intake",
                elevator.goToStationHeight().alongWith(Commands.print("Preparing for intake")));
        NamedCommands.registerCommand(
                "Lower Elevator",
                elevator.elevatorDown().alongWith(Commands.print("Lowering elevator")));
    }

    public void updateAutonomousCommand() {
        System.out.println("Updating auto command!");
        auto = autoChooser.get();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return auto;
    }

    public void setMotorBrake(boolean brake) {}

    /** Update profile states when enabling to prevent PID-only control */
    public void updateMotionProfileStates() {
        elevator.updateProfileStates();
    }
}
