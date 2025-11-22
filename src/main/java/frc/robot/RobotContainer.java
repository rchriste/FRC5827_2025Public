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
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
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

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case COMPETITION:
                // Real robot, instantiate hardware IO implementations
                drive =
                        new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight));

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

                // Disable climb for now
                climb = new Climb(new ClimbIOTalonFX() {});

                endEffector = new EndEffector(new EndEffectorIONeo550());

                elevator =
                        new Elevator(new ElevatorIOTalonFX(), endEffector::isCoralBlockingElevator);
                break;

            case AGNES:
                // Real robot, mk3 swerve, no vision
                drive =
                        new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight));

                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                drive::getPose,
                                new VisionIO() {},
                                new VisionIO() {},
                                new VisionIO() {},
                                new VisionIO() {});

                climb = new Climb(new ClimbIOTalonFX());

                endEffector = new EndEffector(new EndEffectorIO() {});

                elevator = new Elevator(new ElevatorIO() {}, endEffector::isCoralBlockingElevator);
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive =
                        new Drive(
                                new GyroIO() {},
                                new ModuleIOSim(TunerConstants.FrontLeft),
                                new ModuleIOSim(TunerConstants.FrontRight),
                                new ModuleIOSim(TunerConstants.BackLeft),
                                new ModuleIOSim(TunerConstants.BackRight));
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
                // Replayed robot, disable IO implementations
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

        // Change to false to run SysID routines
        this.autoChooser = new AutoChooser(drive, true);

        autoChooser.onChange(this::updateAutonomousCommand);

        superstructure = new Superstructure(drive, endEffector, elevator);

        registerNamedCommands();

        auto =
                Commands.sequence(
                        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(1, 0, 0)), drive)
                                .alongWith(Commands.print("Driving forward!"))
                                .withTimeout(3),
                        Commands.runOnce(() -> drive.stop(), drive)
                                .alongWith(Commands.print("Stopping!")));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));

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
