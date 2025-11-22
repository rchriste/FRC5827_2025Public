package frc.robot.subsystems.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.SwitchableChooser;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

public class AutoChooser extends SubsystemBase {
    private LoggedDashboardChooser<AutoType> autoTypeChooser;
    private LoggedDashboardChooser<StartLocation> startChooser;
    private LoggedDashboardChooser<Integer> autoDelayChooser;
    private LoggedDashboardChooser<Command> autoChooser;
    private SwitchableChooser compAutoChooser;
    private Drive drive;
    private Command auto;
    private Runnable listener;
    private Map<AutoType, List<List<String>>> autoNames;
    private String autoName;
    private boolean comp;
    private AutoType type;

    private enum AutoType {
        GLACIER_PEAK,
        SAMMAMISH;
    }

    private enum StartLocation {
        S1,
        S2,
        S3,
        S4,
        NONE;
    }

    public AutoChooser(Drive drive, boolean comp) {
        this.drive = drive;
        this.auto = Commands.none();
        this.autoNames = new EnumMap<>(AutoType.class);
        this.autoName = "No auto!";
        this.comp = comp;
        this.type = AutoType.SAMMAMISH;
        buildAutoChooser();
    }

    private void buildAutoChooser() {
        // Set up auto routines
        autoDelayChooser = new LoggedDashboardChooser<>("Auto Delay", new SendableChooser<>());
        for (int i = 1; i <= 15; i++) {
            autoDelayChooser.addOption(i + " second delay", i);
        }
        autoTypeChooser = new LoggedDashboardChooser<>("Auto Type", new SendableChooser<>());
        autoTypeChooser.addDefaultOption("Sammamish", AutoType.SAMMAMISH);
        autoTypeChooser.addOption("Glacier Peak", AutoType.GLACIER_PEAK);
        startChooser = new LoggedDashboardChooser<>("Starting Location", new SendableChooser<>());
        autoChooser = new LoggedDashboardChooser<>("SysID Routines", new SendableChooser<>());
        compAutoChooser = new SwitchableChooser("Autos");
        autoChooser.addDefaultOption("None", Commands.none());
        autoDelayChooser.addDefaultOption("No delay", 0);
        startChooser.addDefaultOption("Nowhere", StartLocation.NONE);
        startChooser.addOption("S1", StartLocation.S1);
        startChooser.addOption("S2", StartLocation.S2);
        startChooser.addOption("S3", StartLocation.S3);
        startChooser.addOption("S4", StartLocation.S4);
        List<String> autoNames = AutoBuilder.getAllAutoNames();
        List<List<String>> glacierPeakAutos = new ArrayList<>();
        List<List<String>> sammamishAutos = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            sammamishAutos.add(new ArrayList<>());
        }
        for (int i = 0; i < 4; i++) {
            glacierPeakAutos.add(new ArrayList<>());
        }
        for (String autoName : autoNames) {
            if (!autoName.startsWith("GP - ") && !autoName.startsWith("SAM - ")) {
                continue;
            } else if (autoName.startsWith("GP")) {

                switch (autoName.replaceAll("GP - ", "").substring(0, 2)) {
                    case "S1":
                        glacierPeakAutos.get(0).add(autoName);
                        break;
                    case "S2":
                        glacierPeakAutos.get(1).add(autoName);
                        break;
                    case "S3":
                        glacierPeakAutos.get(2).add(autoName);
                        break;
                    case "S4":
                        glacierPeakAutos.get(3).add(autoName);
                        break;
                }
            } else if (autoName.startsWith("SAM - ")) {
                switch (autoName.replaceAll("SAM - ", "").substring(0, 2)) {
                    case "S1":
                        sammamishAutos.get(0).add(autoName);
                        break;
                    case "S2":
                        sammamishAutos.get(1).add(autoName);
                        break;
                    case "S3":
                        sammamishAutos.get(2).add(autoName);
                        break;
                }
            }
        }
        this.autoNames.put(AutoType.GLACIER_PEAK, glacierPeakAutos);
        this.autoNames.put(AutoType.SAMMAMISH, sammamishAutos);
        for (Map.Entry<AutoType, List<List<String>>> entry : this.autoNames.entrySet()) {
            for (List<String> autos : entry.getValue()) {
                autos.add("No auto!");
            }
        }
        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization",
                DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        autoTypeChooser.getSendableChooser().onChange((s) -> switchAutoType(s));
        startChooser.getSendableChooser().onChange((s) -> switchChooser(s));
    }

    private void switchAutoType(String type) {
        switch (type) {
            case "Glacier Peak":
                this.type = AutoType.GLACIER_PEAK;
                String location = String.valueOf(startChooser.get());
                switch (location) {
                    case "S1":
                        compAutoChooser.setOptions(
                                autoNames.get(this.type).get(0).toArray(new String[0]));
                        break;
                    case "S2":
                        compAutoChooser.setOptions(
                                autoNames.get(this.type).get(1).toArray(new String[0]));
                        break;
                    case "S3":
                        compAutoChooser.setOptions(
                                autoNames.get(this.type).get(2).toArray(new String[0]));
                        break;
                    case "S4":
                    default:
                        compAutoChooser.setOptions(
                                autoNames.get(this.type).get(3).toArray(new String[0]));
                        break;
                }
                break;
            case "Sammamish":
            default:
                this.type = AutoType.SAMMAMISH;
                String autoLocation = String.valueOf(startChooser.get());
                switch (autoLocation) {
                    case "S1":
                        compAutoChooser.setOptions(
                                autoNames.get(this.type).get(0).toArray(new String[0]));
                        break;
                    case "S2":
                        compAutoChooser.setOptions(
                                autoNames.get(this.type).get(1).toArray(new String[0]));
                        break;
                    case "S3":
                    case "S4":
                    default:
                        compAutoChooser.setOptions(
                                autoNames.get(this.type).get(2).toArray(new String[0]));
                        break;
                }
                break;
        }
    }

    private void switchChooser(String location) {
        switch (location) {
            case "S1":
                compAutoChooser.setOptions(
                        this.autoNames.get(this.type).get(0).toArray(new String[0]));
                break;
            case "S2":
                compAutoChooser.setOptions(
                        this.autoNames.get(this.type).get(1).toArray(new String[0]));
                break;
            case "S3":
                compAutoChooser.setOptions(
                        this.autoNames.get(this.type).get(2).toArray(new String[0]));
                break;
            case "S4":
                if (type == AutoType.GLACIER_PEAK) {
                    compAutoChooser.setOptions(
                            this.autoNames.get(this.type).get(3).toArray(new String[0]));
                }
                break;
        }
    }

    public Command get() {
        return auto;
    }

    public void onChange(Runnable listener) {
        this.listener = listener;
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        if (RobotState.isDisabled()) {
            String previousAutoName = autoName;
            if (comp) {
                if (compAutoChooser.get().equals("No auto!")) {
                    autoName = "No auto!";
                } else {
                    autoName = autoDelayChooser.get() + compAutoChooser.get();
                }
            } else {
                autoName = autoDelayChooser.get() + " " + autoChooser.get();
            }

            if (!previousAutoName.equals(autoName)) {
                if (comp) {
                    if (compAutoChooser.get().equals("No auto!")) {
                        auto =
                                Commands.sequence(
                                        Commands.waitSeconds(autoDelayChooser.get()),
                                        Commands.run(
                                                        () ->
                                                                drive.runVelocity(
                                                                        new ChassisSpeeds(1, 0, 0)),
                                                        drive)
                                                .alongWith(Commands.print("Driving forward!"))
                                                .withTimeout(3),
                                        Commands.runOnce(() -> drive.stop(), drive)
                                                .alongWith(Commands.print("Stopping!")));
                    } else {
                        auto =
                                Commands.waitSeconds(autoDelayChooser.get())
                                        .andThen(new PathPlannerAuto(compAutoChooser.get()));
                    }
                } else {
                    auto = Commands.waitSeconds(autoDelayChooser.get()).andThen(autoChooser.get());
                }
                listener.run();
            }
        }
        Logger.recordOutput(
                "PeriodicTime/AutoChooser", (Timer.getFPGATimestamp() - startTime) * 1000);
    }
}
