package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOReal implements EndEffectorIO {
    private PearadoxTalonFX endEffector;

    private Debouncer debouncer;

    public EndEffectorIOReal() {
        endEffector = new PearadoxTalonFX(EndEffectorConstants.END_EFFECTOR_ID, NeutralModeValue.Brake, 60, false);

        BaseStatusSignal.setUpdateFrequencyForAll(
                ArmConstants.UPDATE_FREQ,
                endEffector.getPosition(),
                endEffector.getVelocity(),
                endEffector.getDutyCycle(),
                endEffector.getMotorVoltage(),
                endEffector.getTorqueCurrent(),
                endEffector.getSupplyCurrent(),
                endEffector.getStatorCurrent());

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.2;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        endEffector.getConfigurator().apply(slot0Configs);

        debouncer = new Debouncer(0.125, DebounceType.kRising);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.positionRots = endEffector.getPosition().getValueAsDouble();
        inputs.velocityRps = endEffector.getVelocity().getValueAsDouble();

        inputs.appliedVolts = endEffector.getMotorVoltage().getValueAsDouble();

        inputs.statorCurrent = endEffector.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrent = endEffector.getSupplyCurrent().getValueAsDouble();

        inputs.hasCoral = debouncer.calculate(inputs.statorCurrent > 30);
    }

    @Override
    public void setSpeed(double speed) {
        endEffector.set(speed);
    }
}
