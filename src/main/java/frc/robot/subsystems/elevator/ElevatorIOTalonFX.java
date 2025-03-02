package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.PearadoxTalonFX;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final PearadoxTalonFX elevator;
    private final PearadoxTalonFX elevatorFollower;

    private final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    private final VoltageOut voltageOut =
            new VoltageOut(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);
    private final MotionMagicVoltage motionMagicRequest =
            new MotionMagicVoltage(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);

    // these correspond to linear inches, NOT rotations
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> temp;
    private final StatusSignal<Double> dutyCycle;

    private final StatusSignal<Voltage> followerAppliedVolts;
    private final StatusSignal<Current> followerTorqueCurrent;
    private final StatusSignal<Current> followerSupplyCurrent;
    private final StatusSignal<Temperature> followerTemp;

    public ElevatorIOTalonFX() {

        elevator = new PearadoxTalonFX(
                ElevatorConstants.ELEVATOR_ID,
                ElevatorConstants.MODE,
                ElevatorConstants.CURRENT_LIMIT,
                ElevatorConstants.IS_INVERTED);

        elevatorFollower = new PearadoxTalonFX(
                ElevatorConstants.ELEVATOR_FOLLOWER_ID,
                ElevatorConstants.MODE,
                ElevatorConstants.CURRENT_LIMIT,
                ElevatorConstants.IS_INVERTED);

        elevatorFollower.setControl(new Follower(ElevatorConstants.ELEVATOR_FOLLOWER_ID, true));

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = ElevatorConstants.kG;
        slot0Configs.kS = ElevatorConstants.kS;
        slot0Configs.kV = ElevatorConstants.kV;
        slot0Configs.kA = ElevatorConstants.kA;
        slot0Configs.kP = ElevatorConstants.kP;
        slot0Configs.kI = ElevatorConstants.kI;
        slot0Configs.kD = ElevatorConstants.kD;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MM_CRUISE_VELCOCITY;
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MM_ACCELERATION;

        position = elevator.getPosition();
        velocity = elevator.getVelocity();
        dutyCycle = elevator.getDutyCycle();
        appliedVolts = elevator.getMotorVoltage();
        torqueCurrent = elevator.getTorqueCurrent();
        supplyCurrent = elevator.getSupplyCurrent();
        statorCurrent = elevator.getStatorCurrent();
        temp = elevator.getDeviceTemp();

        followerAppliedVolts = elevatorFollower.getMotorVoltage();
        followerTorqueCurrent = elevatorFollower.getTorqueCurrent();
        followerSupplyCurrent = elevatorFollower.getSupplyCurrent();
        followerTemp = elevatorFollower.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                ElevatorConstants.UPDATE_FREQ,
                position,
                velocity,
                dutyCycle,
                appliedVolts,
                torqueCurrent,
                supplyCurrent,
                statorCurrent);

        BaseStatusSignal.setUpdateFrequencyForAll(
                ElevatorConstants.UPDATE_FREQ,
                followerAppliedVolts,
                followerTorqueCurrent,
                followerSupplyCurrent,
                followerTemp);

        elevator.getConfigurator().apply(talonFXConfigs);
        elevator.optimizeBusUtilization();

        elevatorFollower.getConfigurator().apply(talonFXConfigs);
        elevatorFollower.optimizeBusUtilization();
        elevatorFollower.setControl(new Follower(ElevatorConstants.ELEVATOR_ID, true));
    }

    @Override
    public void updateInputs(ElevatorIOInputAutoLogged inputs) {

        inputs.positionInches = position.getValueAsDouble();
        inputs.velocityInchesPerSec = velocity.getValueAsDouble();

        inputs.appliedVolts = new double[] {appliedVolts.getValueAsDouble(), followerAppliedVolts.getValueAsDouble()};
        inputs.torqueCurrentAmps =
                new double[] {torqueCurrent.getValueAsDouble(), followerTorqueCurrent.getValueAsDouble()};
        inputs.supplyCurrentAmps =
                new double[] {supplyCurrent.getValueAsDouble(), followerSupplyCurrent.getValueAsDouble()};
        inputs.tempCelsius = new double[] {temp.getValueAsDouble(), followerTemp.getValueAsDouble()};
    }

    @Override
    public void setPosition(double positionRots) {
        elevator.setControl(motionMagicRequest.withPosition(positionRots));
    }

    @Override
    public void setVoltage(double volts) {
        elevator.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void setControlConstants(double kG, double kS, double kV, double kA, double kP, double kI, double kD) {
        talonFXConfigs.Slot0.kG = kG;
        talonFXConfigs.Slot0.kS = kS;
        talonFXConfigs.Slot0.kV = kV;
        talonFXConfigs.Slot0.kA = kA;
        talonFXConfigs.Slot0.kP = kP;
        talonFXConfigs.Slot0.kI = kI;
        talonFXConfigs.Slot0.kD = kD;

        elevator.getConfigurator().refresh(talonFXConfigs);
        elevator.getConfigurator().apply(talonFXConfigs);
    }

    @Override
    public void setMotionMagicConstants(double velocity, double acceleration) {
        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = velocity;
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = acceleration;
    }
}
