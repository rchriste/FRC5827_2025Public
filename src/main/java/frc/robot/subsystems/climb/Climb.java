package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    static final double GEAR_REDUCTION = 25.0;
    static final double INTAKE_GEAR_REDUCTION = 3.0;
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs;
    private final LoggedTunableNumber intakeTakeOutPosition =
            new LoggedTunableNumber("Climb/IntakeTakeOutPosition", 0);
    private final LoggedTunableNumber intakePutAwayPosition =
            new LoggedTunableNumber("Climb/IntakePutAwayPosition", 5.3);
    private final LoggedTunableNumber winchReleasePosition =
            new LoggedTunableNumber("Climb/WinchReleasePosition", -1.9);
    private final LoggedTunableNumber winchGrabPosition =
            new LoggedTunableNumber("Climb/WinchGrabPosition", 4.7);

    static final LoggedTunableNumber intakeUpVoltage =
            new LoggedTunableNumber("Climb/IntakeUpVoltage", 2.0);
    static final LoggedTunableNumber intakeOutVoltage =
            new LoggedTunableNumber("Climb/IntakeOutVoltage", 1.0);
    static final LoggedTunableNumber winchOutVoltage =
            new LoggedTunableNumber("Climb/WinchOutVoltage", 7);
    static final LoggedTunableNumber winchInVoltage =
            new LoggedTunableNumber("Climb/WinchInVoltage", -5);

    // So that station intake commands interrupt each other
    private final Subsystem intakeSubsystem = new SubsystemBase("StationIntake") {};

    public Climb(ClimbIO winchIO) {
        this.io = winchIO;
        this.inputs = new ClimbIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        io.updateInputs(inputs);
        Logger.processInputs("Winch", inputs);
        Logger.recordOutput("PeriodicTime/Climb", (Timer.getFPGATimestamp() - startTime) * 1000);
    }

    public Command winchArmOut() {
        return this.startEnd(
                () -> io.setWinchVoltage(winchOutVoltage.getAsDouble()),
                () -> io.setWinchVoltage(0));
    }

    public Command winchArmIn() {
        return this.startEnd(
                () -> io.setWinchVoltage(winchInVoltage.getAsDouble()),
                () -> io.setWinchVoltage(0));
    }

    public Command winchArmDeploy() {
        return this.run(() -> io.setWinchVoltage(winchInVoltage.getAsDouble()))
                .until(
                        () ->
                                inputs.winchPositionRotations
                                        < this.winchReleasePosition.getAsDouble())
                .andThen(this.run(() -> io.setWinchVoltage(winchOutVoltage.getAsDouble())))
                .until(() -> inputs.winchPositionRotations > this.winchGrabPosition.getAsDouble())
                .finallyDo(() -> io.setWinchVoltage(0.0));
    }

    public Command takeOutIntake() {
        return intakeSubsystem
                .startEnd(
                        () -> io.setIntakeVoltage(-intakeOutVoltage.get()),
                        () -> io.setIntakeVoltage(0))
                .until(() -> inputs.intakePositionRotations <= intakeTakeOutPosition.get());
    }

    public Command putAwayIntake() {
        return intakeSubsystem
                .startEnd(
                        () -> io.setIntakeVoltage(intakeUpVoltage.get()),
                        () -> io.setIntakeVoltage(0))
                .until(() -> inputs.intakePositionRotations >= intakePutAwayPosition.get());
    }
}
