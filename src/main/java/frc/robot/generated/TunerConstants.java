package frc.robot.generated;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.swerve.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.LinearVelocity;

import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class TunerConstants {

    // Forward all the static fields with their proper types
    public static final SwerveDrivetrainConstants DrivetrainConstants =
            Constants.currentMode == Mode.COMPETITION
                    ? TunerConstantsJacques.DrivetrainConstants
                    : TunerConstantsAgnes.DrivetrainConstants;

    public static final LinearVelocity kSpeedAt12Volts =
            Constants.currentMode == Mode.COMPETITION
                    ? TunerConstantsJacques.kSpeedAt12Volts
                    : TunerConstantsAgnes.kSpeedAt12Volts;

    public static final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            FrontLeft =
                    Constants.currentMode == Mode.COMPETITION
                            ? TunerConstantsJacques.FrontLeft
                            : TunerConstantsAgnes.FrontLeft;

    public static final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            FrontRight =
                    Constants.currentMode == Mode.COMPETITION
                            ? TunerConstantsJacques.FrontRight
                            : TunerConstantsAgnes.FrontRight;

    public static final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            BackLeft =
                    Constants.currentMode == Mode.COMPETITION
                            ? TunerConstantsJacques.BackLeft
                            : TunerConstantsAgnes.BackLeft;

    public static final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            BackRight =
                    Constants.currentMode == Mode.COMPETITION
                            ? TunerConstantsJacques.BackRight
                            : TunerConstantsAgnes.BackRight;

    // Forward the TunerSwerveDrivetrain inner class
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
        }

        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new,
                    TalonFX::new,
                    CANcoder::new,
                    drivetrainConstants,
                    odometryUpdateFrequency,
                    modules);
        }

        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new,
                    TalonFX::new,
                    CANcoder::new,
                    drivetrainConstants,
                    odometryUpdateFrequency,
                    odometryStandardDeviation,
                    visionStandardDeviation,
                    modules);
        }
    }
}
