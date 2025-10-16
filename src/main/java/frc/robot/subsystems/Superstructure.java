package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.List;
import java.util.Set;

public final class Superstructure extends SubsystemBase {
    private final Drive drive;
    private final EndEffector endEffector;
    private final Elevator elevator;

    // dashboard choosers
    private final LoggedDashboardChooser<List<Pose2d>> scoreDirectionChooser;
    private final LoggedDashboardChooser<ReefLevel> reefLevelSelector;

    // distance from which to start raising the elevator when scoring
    private final LoggedTunableNumber elevatorRaiseDistance =
            new LoggedTunableNumber("Elevator/RaiseDistance", 2.1);

    public Superstructure(Drive drive, EndEffector endEffector, Elevator elevator) {
        this.drive = drive;
        this.endEffector = endEffector;
        this.elevator = elevator;

        scoreDirectionChooser =
                new LoggedDashboardChooser<>("Score Direction", new SendableChooser<>());
        reefLevelSelector = new LoggedDashboardChooser<>("Reef Level", new SendableChooser<>());

        configureDashboardChoosers();
    }

    private void configureDashboardChoosers() {
        reefLevelSelector.addOption("L1", ReefLevel.L1);
        reefLevelSelector.addOption("L2", ReefLevel.L2);
        reefLevelSelector.addOption("L3", ReefLevel.L3);
        reefLevelSelector.addDefaultOption("L4", ReefLevel.L4);

        scoreDirectionChooser.addOption("Left", FieldConstants.Reef.leftBranchPositions2d);
        scoreDirectionChooser.addDefaultOption("Right", FieldConstants.Reef.rightBranchPositions2d);
    }

    private int getNearestZoneIndex() {
        return GeomUtil.getCurrentZone(
                AllianceFlipUtil.apply(drive.getPoseLookahead(0.1)), FieldConstants.Reef.zones);
    }

    // cut-off to start using raw pid instead of trajectory generation
    final Transform2d scoringPointTransform =
            new Transform2d(new Translation2d(0.41, 0.0), Rotation2d.k180deg);

    public Command scoreCommand() {
        return Commands.either(
                        endEffector.drawback(), endEffector.shortDrawback(), elevator::isAtL4)
                .andThen(endEffector.ejectToReef())
                .withTimeout(5.0);
    }

    public Command intakeCommand() {
        return elevator.goToStationHeight()
                .alongWith(endEffector.takeFromIntake())
                .finallyDo(
                        interrupted -> {
                            Commands.either(
                                            endEffector.takeFromIntake().withTimeout(5.0),
                                            Commands.none(),
                                            () ->
                                                    interrupted
                                                            && endEffector
                                                                    .isCoralBlockingElevator())
                                    .andThen(elevator.elevatorDown())
                                    .schedule();
                        });
    }

    public Command elevatorScorePosition() {
        return Commands.defer(
                () ->
                        Commands.waitUntil(
                                        () ->
                                                drive.getPose()
                                                                .getTranslation()
                                                                .getDistance(
                                                                        AllianceFlipUtil.apply(
                                                                                FieldConstants.Reef
                                                                                        .center))
                                                        <= elevatorRaiseDistance.get())
                                .andThen(elevator.elevatorUp(reefLevelSelector.get())),
                Set.of(elevator));
    }

    public Command manualElevatorUp() {
        return Commands.defer(() -> elevator.elevatorUp(reefLevelSelector.get()), Set.of(elevator));
    }

    public Command alignToScore() {
        return Commands.defer(
                () -> {
                    int zone = getNearestZoneIndex();
                    if (zone != -1) {
                        Pose2d targetPose =
                                scoreDirectionChooser
                                        .get()
                                        .get(zone)
                                        .transformBy(scoringPointTransform);
                        return drive.goToPoint(targetPose);
                    } else {
                        return Commands.none();
                    }
                },
                Set.of(drive));
    }

    public Command dealgifyCommand() {
        return Commands.defer(
                () -> {
                    int zone = getNearestZoneIndex();
                    if (zone != -1) {
                        ReefLevel algaeHeight = FieldConstants.Reef.algaeLevelFromZone(zone);
                        Pose2d targetPose =
                                FieldConstants.Reef.centerFaces[getNearestZoneIndex()].transformBy(
                                        scoringPointTransform);
                        return elevator.elevatorUpAlgae(algaeHeight)
                                .alongWith(endEffector.algaeRemoverOpen())
                                .alongWith(drive.goToPoint(targetPose))
                                .andThen(elevator.bumpAlgae(algaeHeight));
                    } else {
                        return Commands.none();
                    }
                },
                Set.of(elevator, drive));
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        if (Constants.currentMode == Constants.Mode.REPLAY) {
            int nearestZoneIndex = getNearestZoneIndex();
            Logger.recordOutput("Drive/Zones", FieldConstants.Reef.zones);
            Logger.recordOutput(
                    "Drive/CurrentZone",
                    nearestZoneIndex == -1
                            ? new Translation2d[] {}
                            : FieldConstants.Reef.zones[nearestZoneIndex]);
        }
        Logger.recordOutput(
                "PeriodicTime/Superstructure", (Timer.getFPGATimestamp() - startTime) * 1000);
    }
}
