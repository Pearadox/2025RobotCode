// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gIntake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ArmConstants;

public class GIntakeIOSim implements GIntakeIO {
    /** Creates a new GIntakeIOSim. */
    private PearadoxTalonFX pivot;

    private TalonFXSimState pivotSimState;

    private TalonFXConfiguration talonFXConfigs;

    private PearadoxTalonFX roller;
    private TalonFXSimState rollerSimState;

    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            GIntakeConstants.PIVOT_GEARING,
            SingleJointedArmSim.estimateMOI(GIntakeConstants.PIVOT_LENGTH, GIntakeConstants.PIVOT_MASS),
            GIntakeConstants.PIVOT_LENGTH,
            Double.NEGATIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            true,
            GIntakeConstants.PIVOT_STARTING_ANGLE);

    private SingleJointedArmSim rollerSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            GIntakeConstants.ROLLER_GEARING,
            SingleJointedArmSim.estimateMOI(0.1, 0.5),
            0.1,
            Double.NEGATIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            true,
            0);

    public GIntakeIOSim() {
        pivot = new PearadoxTalonFX(
                GIntakeConstants.PIVOT_ID,
                NeutralModeValue.Brake,
                GIntakeConstants.PIVOT_CURRENT_LIMIT,
                false); // instantiating a TalonFX object to simulate

        talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.Slot0 = GIntakeConstants.getPivotConfig();

        pivot.getConfigurator().apply(talonFXConfigs.Slot0);
        // PID and FF configurations for the motor

        roller = new PearadoxTalonFX(
                GIntakeConstants.ROLLER_ID,
                NeutralModeValue.Coast,
                GIntakeConstants.ROLLER_CURRENT_LIMIT,
                false); // instantiating another TalonFX object to simulate

        pivotSimState = pivot.getSimState();
        rollerSimState = roller.getSimState();
        // simulation states of said TalonFX objects - THESE ARE SIMULATED MOTORS

        BaseStatusSignal.setUpdateFrequencyForAll(
                ArmConstants.UPDATE_FREQ,
                pivot.getPosition(),
                pivot.getVelocity(),
                pivot.getStatorCurrent(),
                pivot.getSupplyCurrent(),
                pivot.getMotorVoltage(),
                roller.getVelocity(),
                roller.getMotorVoltage());
    }

    public void updateInputs(GIntakeIOInputsAutoLogged inputs) {
        updateSim(); // updates the position and velocities of simulated motors

        inputs.pivotPositionRots = pivot.getPosition().getValueAsDouble();
        inputs.pivotSpeedRps = pivot.getVelocity().getValueAsDouble();

        inputs.pivotStatorCurrent = pivot.getStatorCurrent().getValueAsDouble();
        inputs.pivotSupplyCurrent = pivot.getSupplyCurrent().getValueAsDouble();

        inputs.pivotMotorVoltage = pivot.getMotorVoltage().getValueAsDouble();

        inputs.rollerPositionRots = roller.getPosition().getValueAsDouble();
        inputs.rollerSpeedRps = roller.getVelocity().getValueAsDouble();

        inputs.rollerVoltage = roller.getMotorVoltage().getValueAsDouble();
    }

    public void runPivotPosition(double setpoint) {
        pivot.setControl(new PositionVoltage(setpoint));
    }

    public void runPivotVoltage(double voltage) {
        pivot.setControl(new VoltageOut(voltage));
    }

    public void runRollerVoltage(double voltage) {
        roller.setControl(new VoltageOut(voltage));
    }

    public void updateSim() {
        pivotSimState.setSupplyVoltage(12); // supply voltage is 12 bc 12V batteries

        pivotSim.setInputVoltage(pivotSimState.getMotorVoltage());

        pivotSimState.setRawRotorPosition(
                Units.radiansToRotations(pivotSim.getAngleRads() * GIntakeConstants.PIVOT_GEARING));
        pivotSimState.setRotorVelocity(
                Units.radiansToRotations(pivotSim.getVelocityRadPerSec() * GIntakeConstants.PIVOT_GEARING));

        rollerSimState.setSupplyVoltage(12); // supply voltage is 12 bc 12V batteries

        rollerSim.setInputVoltage(rollerSimState.getMotorVoltage());

        rollerSimState.setRawRotorPosition(
                Units.radiansToRotations(rollerSim.getAngleRads() * GIntakeConstants.ROLLER_GEARING));
        rollerSimState.setRotorVelocity(
                Units.radiansToRotations(rollerSim.getVelocityRadPerSec()) * GIntakeConstants.ROLLER_GEARING);

        pivotSim.update(0.02);
        rollerSim.update(0.02); // updates the simulation every 20ms
    }
}
