package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.subsystems.Elevator.MechVisualizer;

public class ArmIOSim implements ArmIO {
    private SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            ArmConstants.ARM_GEAR_RATIO,
            SimulationConstants.ARM_MOI,
            SimulationConstants.ARM_LENGTH,
            SimulationConstants.MIN_ANGLE,
            SimulationConstants.MAX_ANGLE,
            SimulationConstants.SIMULATE_GRAVITY,
            SimulationConstants.STARTING_ANGLE);

    private PearadoxTalonFX pivot;
    private TalonFXConfiguration talonFXConfigs;
    private TalonFXSimState pivotMotorSimState;

    public ArmIOSim() {
        pivot = new PearadoxTalonFX(
                ArmConstants.ARM_KRAKEN_ID,
                NeutralModeValue.Brake,
                ArmConstants.CURRENT_LIMIT,
                ArmConstants.CURRENT_LIMIT,
                false);

        BaseStatusSignal.setUpdateFrequencyForAll(
                ArmConstants.UPDATE_FREQ,
                pivot.getPosition(),
                pivot.getVelocity(),
                pivot.getDutyCycle(),
                pivot.getMotorVoltage(),
                pivot.getTorqueCurrent(),
                pivot.getSupplyCurrent(),
                pivot.getStatorCurrent());

        pivot.optimizeBusUtilization();

        talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.Voltage.PeakForwardVoltage = 4.5;
        talonFXConfigs.Voltage.PeakReverseVoltage = -4.5;

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = 0; // add enough Gravity Gain just before motor starts moving
        slot0Configs.kS = 0.15; // Add x output to overcome static friction
        slot0Configs.kV = 7.2; // A velocity target of 1 rps results in x output
        slot0Configs.kA = 0; // An acceleration of 1 rps/s requires x output
        slot0Configs.kP = 0.4; // A position error of x rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity error of 1 rps results in x output
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        pivot.getConfigurator().apply(talonFXConfigs);

        pivotMotorSimState = pivot.getSimState();
    }

    @Override
    public void updateInputs(ArmIOInputsAutoLogged inputs) {
        updateSim();

        inputs.positionRots = pivot.getPosition().getValueAsDouble();
        inputs.velocityRps = pivot.getVelocity().getValueAsDouble();

        inputs.appliedVolts = pivot.getMotorVoltage().getValueAsDouble();

        inputs.statorCurrent = pivot.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrent = pivot.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void runPosition(double setpoint, double feedforward) {
        pivot.setControl(new PositionVoltage(-setpoint).withFeedForward(feedforward));
    }

    private void updateSim() {
        pivotMotorSimState.setSupplyVoltage(12);

        armSim.setInputVoltage(pivotMotorSimState.getMotorVoltage());
        armSim.update(0.02);

        pivotMotorSimState.setRawRotorPosition(
                (Units.radiansToRotations(armSim.getAngleRads() - SimulationConstants.STARTING_ANGLE))
                        * ArmConstants.ARM_GEAR_RATIO);

        pivotMotorSimState.setRotorVelocity(
                Units.radiansToRotations(armSim.getVelocityRadPerSec() * ArmConstants.ARM_GEAR_RATIO));

        MechVisualizer.getInstance().updateArmAngle(armSim.getAngleRads());
    }
}
