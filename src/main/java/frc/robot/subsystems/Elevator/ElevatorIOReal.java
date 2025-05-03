package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {
    private PearadoxTalonFX elevatorMotor;
    private PearadoxTalonFX elevatorFollower;
    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    public ElevatorIOReal() {
        elevatorMotor = new PearadoxTalonFX(
                ElevatorConstants.ELEVATOR_ID,
                ElevatorConstants.MODE,
                ElevatorConstants.CURRENT_LIMIT,
                ElevatorConstants.IS_INVERTED);

        elevatorFollower = new PearadoxTalonFX(
                ElevatorConstants.ELEVATOR_FOLLOWER_ID,
                ElevatorConstants.MODE,
                ElevatorConstants.CURRENT_LIMIT,
                ElevatorConstants.IS_INVERTED);

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = ElevatorConstants.kG; // add enough Gravity Gain just before motor starts moving
        slot0Configs.kS = ElevatorConstants.kS; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = ElevatorConstants.kV; // A velocity target of 1 rps results in 0.1 V output
        slot0Configs.kA = ElevatorConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = ElevatorConstants.kP; // A position error of 2.5 rotations results in 12 V output, prev 4.8
        slot0Configs.kI = ElevatorConstants.kI; // no output for integrated error
        slot0Configs.kD = ElevatorConstants.kD; // A velocity error of 1 rps results in 0.1 V output

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity =
                ElevatorConstants.MM_CRUISE_VELCOCITY_UP; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration =
                ElevatorConstants.MM_ACCELERATION_UP; // Target acceleration of 160 rps/s (0.5 seconds)
        // (not sure if needed - > ) motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1
        // seconds)

        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                33 * ElevatorConstants.GEAR_RATIO / (Math.PI * ElevatorConstants.PULLEY_DIAMETER);
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        talonFXConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        elevatorMotor.getConfigurator().apply(talonFXConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
                ElevatorConstants.UPDATE_FREQ,
                elevatorMotor.getPosition(),
                elevatorMotor.getVelocity(),
                elevatorMotor.getDutyCycle(),
                elevatorMotor.getMotorVoltage(),
                elevatorMotor.getTorqueCurrent(),
                elevatorMotor.getSupplyCurrent(),
                elevatorMotor.getStatorCurrent());

        elevatorMotor.optimizeBusUtilization();

        elevatorFollower.getConfigurator().apply(talonFXConfigs);
        elevatorFollower.optimizeBusUtilization();
        elevatorFollower.setControl(new Follower(ElevatorConstants.ELEVATOR_ID, true));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRots = elevatorMotor.getPosition().getValueAsDouble();
        inputs.velocityRps = elevatorMotor.getVelocity().getValueAsDouble();

        inputs.appliedVolts = elevatorMotor.getMotorVoltage().getValueAsDouble();

        inputs.statorCurrent = elevatorMotor.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrent = elevatorMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void reachGoal(double setpoint) {
        elevatorMotor.setControl(motionMagicRequest.withPosition(setpoint));
    }

    @Override
    public void setSpeed(double speed) {
        elevatorMotor.set(speed);
    }

    @Override
    public void setPosition(double position) {
        elevatorMotor.setPosition(position);
    }
}
