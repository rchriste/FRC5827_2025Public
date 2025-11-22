package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class EndEffectorIOSim implements EndEffectorIO {
    private final DCMotorSim endEffectorSim;
    private final DCMotor endEffectorMotor = DCMotor.getNeo550(1);
    private double requestedVoltage = 0.0;
    private double requestedAlgaeServoPosition = 0.0;
    private final BooleanSubscriber pickupSubscriber;
    private final BooleanSubscriber dropoffSubscriber;

    public EndEffectorIOSim() {
        endEffectorSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(endEffectorMotor, 0.001, 1),
                        endEffectorMotor);
        // Yo, NetworkTablesInstance class, give me your default table.
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // Table, please give me the boolean topic for this string
        BooleanTopic pickupTopic = inst.getBooleanTopic("AdvantageKit/endEffectorSim/pickup");
        BooleanTopic dropoffTopic = inst.getBooleanTopic("AdvantageKit/endEffectorSim/dropoff");
        pickupTopic.publish();
        dropoffTopic.publish();
        pickupSubscriber = pickupTopic.subscribe(false);
        dropoffSubscriber = dropoffTopic.subscribe(false);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        endEffectorSim.setInputVoltage(MathUtil.clamp(requestedVoltage, -12, 12));
        endEffectorSim.update(0.02);
        inputs.endEffectorConnected = true;
        inputs.endEffectorPositionRad = endEffectorSim.getAngularPositionRad();
        inputs.endEffectorVelocityRadPerSec = endEffectorSim.getAngularVelocityRadPerSec();
        inputs.endEffectorAppliedVolts = requestedVoltage;
        inputs.endEffectorCurrentAmps = endEffectorSim.getCurrentDrawAmps();
        inputs.dropoff = this.dropoffSubscriber.get();
        inputs.pickup = this.pickupSubscriber.get();
        inputs.algaeServoRequestedPosition = requestedAlgaeServoPosition;
    }

    @Override
    public void setEndEffectorVoltage(double voltageInVolts) {
        requestedVoltage = voltageInVolts;
    }

    @Override
    public void setAlgaeServoPosition(double newPosition) {
        requestedAlgaeServoPosition = newPosition;
    }
}
