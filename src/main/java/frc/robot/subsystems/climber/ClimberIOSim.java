package frc.robot.subsystems.climber;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ClimbConstants;

public class ClimberIOSim extends ClimberIOReal {
    private SingleJointedArmSim climberSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            ClimbConstants.GEAR_RATIO,
            ClimbConstants.MOI,
            ClimbConstants.CLIMBER_LENGTH,
            ClimbConstants.MIN_ANGLE,
            ClimbConstants.MAX_ANGLE,
            false,
            ClimbConstants.STARTING_ANGLE);

    private TalonFXSimState climberSimState = climbMotor.getSimState();

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        super.updateInputs(inputs);

        climberSimState.setSupplyVoltage(12);

        climberSim.setInputVoltage(climberSimState.getMotorVoltage());
        climberSim.update(0.02);

        climberSimState.setRawRotorPosition(
                Units.radiansToRotations(climberSim.getAngleRads() - ClimbConstants.STARTING_ANGLE)
                        * ClimbConstants.GEAR_RATIO);

        climberSimState.setRotorVelocity(
                Units.radiansToRotations(climberSim.getVelocityRadPerSec() * ClimbConstants.GEAR_RATIO));
    }
}
