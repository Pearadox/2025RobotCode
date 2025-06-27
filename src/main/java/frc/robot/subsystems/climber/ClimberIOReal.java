package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ClimbConstants;

public class ClimberIOReal implements ClimberIO {
    protected PearadoxTalonFX climbMotor;

    public ClimberIOReal() {
        climbMotor = new PearadoxTalonFX(ClimbConstants.CLIMB_MOTOR_ID, NeutralModeValue.Brake, 50, false);

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = 0;
        slot0Configs.kS = 0;
        slot0Configs.kV = 0;
        slot0Configs.kA = 0;
        slot0Configs.kP = 0.25;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        climbMotor.getConfigurator().apply(slot0Configs);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.positionRots = climbMotor.getPosition().getValueAsDouble();
        inputs.velocityRps = climbMotor.getVelocity().getValueAsDouble();
        inputs.appliedVolts = climbMotor.getMotorVoltage().getValueAsDouble();
        inputs.statorCurrent = climbMotor.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrent = climbMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void runPosition(double setpoint) {
        // climbMotor.setControl(new PositionVoltage(setpoint));
        climbMotor.set(0.0);
    }

    @Override
    public void setSpeed(double speed) {
        // climbMotor.set(speed);
        climbMotor.set(0.0);
    }

    @Override
    public void zeroPosition() {
        climbMotor.setPosition(0);
    }
}
