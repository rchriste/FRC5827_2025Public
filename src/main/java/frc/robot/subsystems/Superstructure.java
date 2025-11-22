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

    // Dashboard choosers:
    // LoggedDashboardChooser writes to NetworkTables and reads selections back. Students can change
    // these on the dashboard without redeploying code.
    private final LoggedDashboardChooser<List<Pose2d>> scoreDirectionChooser;
    private final LoggedDashboardChooser<ReefLevel> reefLevelSelector;

    // Distance from which to start raising the elevator when scoring.
    // Starting the lift early makes scoring faster, but starting too early risks tipping or
    // collisions. Because this is a LoggedTunableNumber, we can tune it live and log the value.
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
        // Reef level is which "peg height" we want for coral.
        reefLevelSelector.addOption("L1", ReefLevel.L1);
        reefLevelSelector.addOption("L2", ReefLevel.L2);
        reefLevelSelector.addOption("L3", ReefLevel.L3);
        reefLevelSelector.addDefaultOption("L4", ReefLevel.L4);

        // Score direction lets the driver choose left vs right branch.
        // We still auto-pick the closest zone, but this chooser decides which side within the zone.
        scoreDirectionChooser.addOption("Left", FieldConstants.Reef.leftBranchPositions2d);
        scoreDirectionChooser.addDefaultOption("Right", FieldConstants.Reef.rightBranchPositions2d);
    }

    private int getNearestZoneIndex() {
        // Find the closest reef "zone" to where the robot will be in ~0.1s.
        // This helps the driver a lot: the robot automatically targets the nearest scoring area
        // instead of forcing the driver to pick exact pegs.
        return GeomUtil.getCurrentZone(
                AllianceFlipUtil.apply(drive.getPoseLookahead(0.1)), FieldConstants.Reef.zones);
    }

    // cut-off to start using raw pid instead of trajectory generation
    final Transform2d scoringPointTransform =
            new Transform2d(new Translation2d(0.41, 0.0), Rotation2d.k180deg);

    public Command scoreCommand() {
        // Decide between full or short drawback based on elevator height, then eject coral.
        return Commands.either(
                        endEffector.drawback(), endEffector.shortDrawback(), elevator::isAtL4)
                .andThen(endEffector.ejectToReef())
                .withTimeout(5.0);
    }

    public Command intakeCommand() {
        // Intake sequence:
        //  1) lower elevator to station height
        //  2) spin intake rollers
        //  3) when finished, if we were interrupted AND coral is blocking elevator, keep intaking
        //     briefly and then lower elevator for safety.
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
        // Defer means "wait until the command is scheduled to decide what to do".
        // We wait until the robot is close enough to the reef, then raise to the selected level.
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
        // Auto-align to the nearest zone and selected left/right branch.
        // If no zone is nearby, do nothing.
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
        // Similar to scoring, but targets the algae height for the current zone.
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
            // In replay, log the reef zones and our selected zone so we can visualize decisions in
            // AdvantageScope.
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
