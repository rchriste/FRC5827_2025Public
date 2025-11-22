package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class ClimbIOTalonFX implements ClimbIO {

    private static final double CURRENT_LIMIT = 40.0;

    private final TalonFX winchMotor;
    private final TalonFX intakeMotor;
    private final TalonFXConfiguration config;
    public StatusSignal<Angle> winchClimbPosition;
    public StatusSignal<Voltage> winchClimbAppliedVolts;
    public StatusSignal<Current> winchClimbCurrentAmps;
    public StatusSignal<Temperature> winchMotorTemperature;
    public StatusSignal<Angle> intakePosition;
    public StatusSignal<Voltage> intakeAppliedVolts;
    public StatusSignal<Current> intakeCurrentAmps;
    public StatusSignal<Temperature> intakeTemperature;

    private final VoltageOut climbVoltageOut = new VoltageOut(0.0);
    private final VoltageOut intakeVoltageOut = new VoltageOut(0.0);

    public ClimbIOTalonFX() {
        this.winchMotor = new TalonFX(Constants.climbOpenCanbus_ID);
        intakeMotor = new TalonFX(Constants.stationIntakeCan_ID);
        config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        config.Feedback.SensorToMechanismRatio = Climb.GEAR_REDUCTION;

        PhoenixUtil.tryUntilOk(5, () -> this.winchMotor.getConfigurator().apply(config, 0.25));

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = Climb.INTAKE_GEAR_REDUCTION;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.tryUntilOk(5, () -> this.intakeMotor.getConfigurator().apply(config, 0.25));

        winchClimbPosition = winchMotor.getPosition();
        winchClimbAppliedVolts = winchMotor.getMotorVoltage();
        winchClimbCurrentAmps = winchMotor.getSupplyCurrent();
        winchMotorTemperature = winchMotor.getDeviceTemp();

        intakePosition = intakeMotor.getPosition();
        intakeAppliedVolts = intakeMotor.getMotorVoltage();
        intakeCurrentAmps = intakeMotor.getSupplyCurrent();
        intakeTemperature = intakeMotor.getDeviceTemp();

        // 50.0 hz = 20 ms (same as periodic method)
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                winchClimbPosition,
                winchClimbAppliedVolts,
                winchClimbCurrentAmps,
                winchMotorTemperature,
                intakePosition,
                intakeAppliedVolts,
                intakeCurrentAmps,
                intakeTemperature);
        ParentDevice.optimizeBusUtilizationForAll(winchMotor, intakeMotor);

        winchMotor.setPosition(0);
        intakeMotor.setPosition(0);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        var winchStatus =
                BaseStatusSignal.refreshAll(
                        winchClimbPosition,
                        winchClimbAppliedVolts,
                        winchClimbCurrentAmps,
                        winchMotorTemperature);
        var intakeStatus =
                BaseStatusSignal.refreshAll(
                        intakePosition, intakeAppliedVolts, intakeCurrentAmps, intakeTemperature);
        inputs.winchConnected = winchStatus.isOK();
        inputs.winchPositionRotations = winchClimbPosition.getValueAsDouble();
        inputs.winchAppliedVolts = winchClimbAppliedVolts.getValueAsDouble();
        inputs.winchCurrentAmps = winchClimbCurrentAmps.getValueAsDouble();
        inputs.winchTemperatureCelsius = winchMotorTemperature.getValueAsDouble();

        inputs.intakeConnected = intakeStatus.isOK();
        inputs.intakePositionRotations = intakePosition.getValueAsDouble();
        inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
        inputs.intakeCurrentAmps = intakeCurrentAmps.getValueAsDouble();
        inputs.intakeTemperatureCelsius = intakeTemperature.getValueAsDouble();
    }

    @Override
    public void setWinchVoltage(double voltage) {
        winchMotor.setControl(climbVoltageOut.withOutput(voltage));
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        intakeMotor.setControl(intakeVoltageOut.withOutput(voltage));
    }
}
