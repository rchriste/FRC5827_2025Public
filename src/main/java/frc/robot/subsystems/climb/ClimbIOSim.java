package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim implements ClimbIO {

    private final DCMotorSim climbSim;
    private final DCMotor winchMotor = DCMotor.getKrakenX60(1);
    private double winchRequestedVoltage = 0;
    private double requestedIntakePosition = 0;

    public ClimbIOSim() {
        climbSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(winchMotor, 0.001, 1), winchMotor);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        climbSim.setInputVoltage(MathUtil.clamp(winchRequestedVoltage, -12, 12));
        climbSim.update(0.02);
    }

    @Override
    public void setWinchVoltage(double voltage) {
        winchRequestedVoltage = voltage;
    }
}
