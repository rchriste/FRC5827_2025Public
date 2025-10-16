package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    static final LoggedTunableNumber LEFT_ROLLER_VOLTAGE =
            new LoggedTunableNumber("Intake/Left Roller Voltage", 5.0);
    static final LoggedTunableNumber RIGHT_ROLLER_VOLTAGE =
            new LoggedTunableNumber("Intake/Right Roller Voltage", 5.0);

    static final LoggedTunableNumber ROLLER_PASS_VOLTAGE =
            new LoggedTunableNumber("Intake/Roller Pass Voltage", 2.0);

    static final double PIVOT_TOLERANCE_RAD = Units.degreesToRadians(5);
    static final double PIVOT_VELOCITY_TOLERANCE_RAD_PER_SEC = Units.degreesToRadians(5);

    // PID constants
    // for motion profiling, I found P=1, D=0.01 to work decently with well-tuned feedforward
    static final LoggedTunableNumber PIVOT_kP = new LoggedTunableNumber("Intake/Pivot kP", 100);
    static final LoggedTunableNumber PIVOT_kD = new LoggedTunableNumber("Intake/Pivot kD", 2.5);

    // feedforward constants
    static final LoggedTunableNumber PIVOT_kS = new LoggedTunableNumber("Intake/Pivot kS", 0.0);
    static final LoggedTunableNumber PIVOT_kV = new LoggedTunableNumber("Intake/Pivot kV", 4.0);
    static final LoggedTunableNumber PIVOT_kA = new LoggedTunableNumber("Intake/Pivot kA", 0.7);
    static final LoggedTunableNumber PIVOT_kG = new LoggedTunableNumber("Intake/Pivot kG", 0.24);

    static final LoggedTunableNumber PIVOT_MAX_VELOCITY =
            new LoggedTunableNumber("Intake/Max Velocity", 1.0);
    static final LoggedTunableNumber PIVOT_MAX_ACCELERATION =
            new LoggedTunableNumber("Intake/Max Acceleration", 2.5);

    // target points, 0 is always horizontal by convention
    /** from center of mass, degrees */
    private static final double COM_ANGLE_OFFSET = -13.615656;

    static final Angle PIVOT_STOW_ANGLE = Degrees.of(146 + COM_ANGLE_OFFSET);
    static final LoggedTunableNumber PIVOT_RETRACT_ANGLE_RAD =
            new LoggedTunableNumber("Intake/PivotRetractPosition", 1.93);
    static final LoggedTunableNumber PIVOT_DEPLOY_ANGLE_RAD =
            new LoggedTunableNumber("Intake/PivotDeployPosition", 0.25);

    // starting state
    static final Angle PIVOT_STARTING_ANGLE = PIVOT_STOW_ANGLE;

    // pivot max/min angles
    private static final double PIVOT_LIMIT_TOLERANCE_RAD = Units.degreesToRadians(3);
    static final Angle PIVOT_MAX_ANGLE =
            Radians.of(PIVOT_STOW_ANGLE.in(Radians) + PIVOT_LIMIT_TOLERANCE_RAD);
    static final Angle PIVOT_MIN_ANGLE =
            Radians.of(PIVOT_DEPLOY_ANGLE_RAD.get() - PIVOT_LIMIT_TOLERANCE_RAD);

    // gear ratio between motor and encoder
    static final double PIVOT_GEAR_RATIO = 128.57142857142858;

    static final LoggedTunableNumber HAS_CORAL_CURRENT_AMPS =
            new LoggedTunableNumber("Intake/Has Coral Current", 35);

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;
    private final LoggedTunableNumber pivotVoltage =
            new LoggedTunableNumber("Intake/Pivot Voltage", 1.0);
    private final ArmFeedforward pivotFeedforward;
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State profileCurrentState, profileGoalState;
    private boolean doMotionProfiling = false;
    private boolean atSetpoint = false;

    public Intake(IntakeIO io) {
        this.io = io;
        this.inputs = new IntakeIOInputsAutoLogged();

        pivotFeedforward =
                new ArmFeedforward(PIVOT_kS.get(), PIVOT_kG.get(), PIVOT_kV.get(), PIVOT_kA.get());
        motionProfile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                PIVOT_MAX_VELOCITY.get(), PIVOT_MAX_ACCELERATION.get()));

        profileCurrentState = new TrapezoidProfile.State(PIVOT_STARTING_ANGLE.in(Rotations), 0.0);
        profileGoalState = new TrapezoidProfile.State(PIVOT_STARTING_ANGLE.in(Rotations), 0.0);
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        Logger.recordOutput(
                "Intake/Current Profile Position",
                Units.rotationsToRadians(profileCurrentState.position));
        Logger.recordOutput(
                "Intake/Current Profile Velocity",
                Units.rotationsToRadians(profileCurrentState.velocity));

        if (doMotionProfiling) {
            var profileNewState =
                    motionProfile.calculate(0.02, profileCurrentState, profileGoalState);
            double feedforwardVoltage =
                    pivotFeedforward.calculateWithVelocities(
                            profileCurrentState.position,
                            profileCurrentState.velocity,
                            profileNewState.velocity);
            io.setPivotState(
                    profileCurrentState.position, profileCurrentState.velocity, feedforwardVoltage);
            profileCurrentState = profileNewState;
        }

        atSetpoint =
                MathUtil.isNear(
                                Units.rotationsToRadians(profileGoalState.position),
                                inputs.pivotPositionRadians,
                                PIVOT_TOLERANCE_RAD)
                        && MathUtil.isNear(
                                Units.rotationsToRadians(profileGoalState.velocity),
                                inputs.pivotVelocityRadPerSec,
                                PIVOT_VELOCITY_TOLERANCE_RAD_PER_SEC);
        Logger.recordOutput("Intake/At Setpoint", atSetpoint);

        if (PIVOT_kP.hasChanged(this.hashCode()) || PIVOT_kD.hasChanged(this.hashCode())) {
            io.updatePID(PIVOT_kP.get(), PIVOT_kD.get());
        }

        if (PIVOT_MAX_VELOCITY.hasChanged(this.hashCode())
                || PIVOT_MAX_ACCELERATION.hasChanged(this.hashCode())) {
            motionProfile =
                    new TrapezoidProfile(
                            new TrapezoidProfile.Constraints(
                                    PIVOT_MAX_VELOCITY.get(), PIVOT_MAX_ACCELERATION.get()));
        }

        if (PIVOT_kS.hasChanged(this.hashCode())
                || PIVOT_kV.hasChanged(this.hashCode())
                || PIVOT_kA.hasChanged(this.hashCode())
                || PIVOT_kG.hasChanged(this.hashCode())) {
            pivotFeedforward.setKs(PIVOT_kS.get());
            pivotFeedforward.setKv(PIVOT_kV.get());
            pivotFeedforward.setKa(PIVOT_kA.get());
            pivotFeedforward.setKg(PIVOT_kG.get());
        }
        Logger.recordOutput("PeriodicTime/Intake", (Timer.getFPGATimestamp() - startTime) * 1000);
    }

    public void updateProfileStates() {
        profileCurrentState.position = Units.radiansToRotations(inputs.pivotPositionRadians);
        profileCurrentState.velocity = Units.radiansToRotations(inputs.pivotVelocityRadPerSec);
    }

    public void setBrakeMode(boolean brake) {
        io.setPivotBrakeMode(brake);
    }

    public Command stopArm() {
        return this.runOnce(
                () -> {
                    io.setPivotVoltage(0);
                    doMotionProfiling = false;
                });
    }

    public Command armUp() {
        return this.startEnd(
                () -> {
                    io.setPivotVoltage(pivotVoltage.get());
                    doMotionProfiling = false;
                },
                () -> io.setPivotVoltage(0));
    }

    public Command armDown() {
        return this.startEnd(
                () -> {
                    io.setPivotVoltage(-pivotVoltage.get());
                    doMotionProfiling = false;
                },
                () -> io.setPivotVoltage(0));
    }

    public Command stowArm() {
        return this.runOnce(
                        () -> {
                            profileGoalState.position = PIVOT_STOW_ANGLE.in(Rotations);
                            atSetpoint = false;
                            doMotionProfiling = true;
                        })
                .andThen(Commands.waitUntil(() -> atSetpoint));
    }

    public Command retractArm() {
        return this.runOnce(
                        () -> {
                            profileGoalState.position =
                                    Units.radiansToRotations(PIVOT_RETRACT_ANGLE_RAD.get());
                            atSetpoint = false;
                            doMotionProfiling = true;
                        })
                .andThen(Commands.waitUntil(() -> atSetpoint));
    }

    public Command deployArm() {
        return this.runOnce(
                        () -> {
                            profileGoalState.position =
                                    Units.radiansToRotations(PIVOT_DEPLOY_ANGLE_RAD.get());
                            atSetpoint = false;
                            doMotionProfiling = true;
                        })
                .andThen(Commands.waitUntil(() -> atSetpoint));
    }

    public Command rollerPass() {
        return this.startEnd(
                () -> io.setRollerVoltage(ROLLER_PASS_VOLTAGE.get(), ROLLER_PASS_VOLTAGE.get()),
                () -> io.setRollerVoltage(0, 0));
    }

    public Command rollerReverse() {
        return this.startEnd(
                () -> io.setRollerVoltage(-LEFT_ROLLER_VOLTAGE.get(), -RIGHT_ROLLER_VOLTAGE.get()),
                () -> io.setRollerVoltage(0, 0));
    }

    /** DOES NOT REQUIRE THIS SUBSYSTEM */
    private Command rollerIntakeCoral() {
        return Commands.startEnd(
                        () ->
                                io.setRollerVoltage(
                                        LEFT_ROLLER_VOLTAGE.get(), RIGHT_ROLLER_VOLTAGE.get()),
                        () -> io.setRollerVoltage(0, 0))
                .withDeadline(
                        Commands.waitUntil(
                                        () ->
                                                inputs.hasCoral
                                                        && inputs.leftMotorAmps >= 38
                                                        && inputs.rightMotorAmps >= 38)
                                .andThen(Commands.waitSeconds(0.1)));
    }

    public Command intakeCoral() {
        return deployArm().alongWith(rollerIntakeCoral()).andThen(retractArm());
    }

    public boolean isPivotRetracted() {
        return profileGoalState.position == Units.radiansToRotations(PIVOT_RETRACT_ANGLE_RAD.get())
                && atSetpoint;
    }

    public boolean hasCoral() {
        return inputs.hasCoral;
    }
}
