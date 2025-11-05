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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SimulationConstants;

public class GIntakeIOSim implements GIntakeIO {
    /** Creates a new GIntakeIOSim. */
    private PearadoxTalonFX pivot;

    private TalonFXSimState pivotSimState;

    private TalonFXConfiguration TalonFXConfigs;

    private PearadoxTalonFX roller;
    private TalonFXSimState rollerSimState;

    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            IntakeConstants.PIVOT_GEARING,
            SimulationConstants.GINTAKE_MOI,
            SimulationConstants.GINTAKE_LENGTH,
            SimulationConstants.MIN_ANGLE,
            SimulationConstants.MAX_ANGLE,
            SimulationConstants.SIMULATE_GRAVITY,
            SimulationConstants.GINTAKE_STARTING_ANGLE);

    // private SingleJointedArmSim rollerSim = new SingleJointedArmSim(
    //     DCMotor.getKrakenX60(1),
    //     IntakeConstants.ROLLER_GEARING,
    //     SingleJointedArmSim.estimateMOI(0.1, 0.5),
    //     0.1,
    //     SimulationConstants.MIN_ANGLE,
    //     SimulationConstants.MAX_ANGLE,
    //     SimulationConstants.SIMULATE_GRAVITY,
    //     0
    // );

    public GIntakeIOSim() {
        pivot = new PearadoxTalonFX(
                IntakeConstants.PIVOT_ID,
                NeutralModeValue.Brake,
                IntakeConstants.PIVOT_CURRENT_LIMIT,
                false); // instantiating a TalonFX object to simulate

        TalonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = TalonFXConfigs.Slot0;

        slot0Configs.kP = 0.1;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.1;

        pivot.getConfigurator().apply(slot0Configs);
        // PID and FF configurations for the motor

        roller = new PearadoxTalonFX(
                IntakeConstants.ROLLER_ID,
                NeutralModeValue.Coast,
                IntakeConstants.ROLLER_CURRENT_LIMIT,
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

        inputs.positionRots = pivot.getPosition().getValueAsDouble();
        inputs.rollerSpeedRps = roller.getVelocity().getValueAsDouble();
        // updates inputs based on updated positions

        inputs.pivotStatorCurrent = pivot.getStatorCurrent().getValueAsDouble();
        inputs.pivotSupplyCurrent = pivot.getSupplyCurrent().getValueAsDouble();
    }

    public void runPosition(double setpoint, boolean isIntaking, double feedForward) {
        PositionVoltage pivotPositionRequest = new PositionVoltage(setpoint);

        VoltageOut rollerVoltageOut =
                new VoltageOut(isIntaking ? IntakeConstants.ROLLER_INTAKE_SPEED : IntakeConstants.ROLLER_OUTAKE_SPEED);

        pivot.setControl(pivotPositionRequest);
        roller.setControl(rollerVoltageOut);
    }

    public void updateSim() {
        pivotSimState.setSupplyVoltage(12);
        rollerSimState.setSupplyVoltage(12);

        pivotSim.setInputVoltage(pivotSimState.getMotorVoltage());
        // rollerSim.setInputVoltage(rollerSimState.getMotorVoltage());
        // sets mech2d simulation's input voltage to motor's simulated applied voltage
        pivotSim.update(0.02); // updates the simulation every 20ms

        // apparently the TalonFXSimState isn't smart enough to solve for it's own position or velocity
        // we need to tell it where it is/how fast it's going
        pivotSimState.setRawRotorPosition(Units.radiansToRotations(pivotSim.getAngleRads()));
        pivotSimState.setRotorVelocity(Units.radiansToRotations(pivotSim.getVelocityRadPerSec()));

        // rollerSimState.setRotorVelocity(Units.radiansToRotations(pivotSim.getVelocityRadPerSec()));
    }
}
