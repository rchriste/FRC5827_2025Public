package frc.robot.subsystems.endEffector;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.Constants;
import frc.robot.util.SparkUtil;

import java.util.function.DoubleSupplier;

public class EndEffectorIONeo550 implements EndEffectorIO {
    private final SparkMax endEffectorMotor;
    private final RelativeEncoder endEffectorEncoder;
    private final Debouncer connectionDebouncer = new Debouncer(0.5);

    private final DigitalInput pickup;
    private final DigitalInput dropoff;
    private final Servo algaeServo = new Servo(this.ALGAE_SERVO_PORT);

    private final int PICK_UP_SENSOR = 1; // Labled with the label maker
    private final int DROP_OFF_SENSOR = 0;
    private final int ALGAE_SERVO_PORT = 9;

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        SparkUtil.sparkStickyFault = false;
        SparkUtil.ifOk(
                endEffectorMotor,
                endEffectorEncoder::getPosition,
                (value) -> inputs.endEffectorPositionRad = Units.rotationsToRadians(value));
        SparkUtil.ifOk(
                endEffectorMotor,
                endEffectorEncoder::getVelocity,
                (value) ->
                        inputs.endEffectorVelocityRadPerSec =
                                Units.rotationsPerMinuteToRadiansPerSecond(value));
        SparkUtil.ifOk(
                endEffectorMotor,
                new DoubleSupplier[] {
                    endEffectorMotor::getAppliedOutput, endEffectorMotor::getBusVoltage
                },
                (values) -> inputs.endEffectorAppliedVolts = values[0] * values[1]);
        SparkUtil.ifOk(
                endEffectorMotor,
                endEffectorMotor::getOutputCurrent,
                (value) -> inputs.endEffectorCurrentAmps = value);
        SparkUtil.ifOk(
                endEffectorMotor,
                endEffectorMotor::getMotorTemperature,
                (value) -> inputs.motorTempCelsius = value);
        inputs.endEffectorConnected = connectionDebouncer.calculate(!SparkUtil.sparkStickyFault);

        // Sensor is if it sees voltage so it needs to be reversed to be what is expected
        inputs.dropoff = !this.dropoff.get();
        // Sensor is if it sees voltage so it needs to be reversed to be what is expected
        inputs.pickup = !this.pickup.get();

        inputs.algaeServoRequestedPosition = algaeServo.get();
    }

    public EndEffectorIONeo550() {
        this.endEffectorMotor = new SparkMax(Constants.endEffectorCanbus_ID, MotorType.kBrushless);
        this.endEffectorEncoder = endEffectorMotor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        // brake prevents overshoot when coasting
        config.idleMode(IdleMode.kBrake)
                .voltageCompensation(12)
                .smartCurrentLimit(100)
                .inverted(true);
        config.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
        config.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        SparkUtil.tryUntilOk(
                this.endEffectorMotor,
                5,
                () ->
                        this.endEffectorMotor.configure(
                                config,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters));

        this.pickup = new DigitalInput(PICK_UP_SENSOR);
        this.dropoff = new DigitalInput(DROP_OFF_SENSOR);
    }

    @Override
    public void setEndEffectorVoltage(double voltageInVolts) {
        endEffectorMotor.setVoltage(voltageInVolts);
    }

    /* position is a value between 0.0 .. 1.0 */
    @Override
    public void setAlgaeServoPosition(double newPosition) {
        algaeServo.set(newPosition);
    }
}
