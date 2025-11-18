// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gIntake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ArmConstants;

public class GIntakeIOReal implements GIntakeIO {

    private PearadoxTalonFX pivot;
    private TalonFXConfiguration talonFXConfigs;

    private PearadoxTalonFX roller;

    /** Creates a new GIntakeIOReal. */
    public GIntakeIOReal() {
        pivot = new PearadoxTalonFX(
                GIntakeConstants.PIVOT_ID, NeutralModeValue.Brake, GIntakeConstants.PIVOT_CURRENT_LIMIT, false);

        talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.Slot0 = GIntakeConstants.getPivotConfig();

        pivot.getConfigurator().apply(talonFXConfigs.Slot0);

        roller = new PearadoxTalonFX(
                GIntakeConstants.ROLLER_ID, NeutralModeValue.Coast, GIntakeConstants.ROLLER_CURRENT_LIMIT, false);

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

    // no getInstance function because instances are dependent on whether SIM or REAL - handled in RobotContainer

    public void updateInputs(GIntakeIOInputsAutoLogged inputs) {
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
        PositionVoltage pivotPositionRequest = new PositionVoltage(setpoint);
        // PositionVoltage is a control setting that sets a motor's desired position and applies voltage based on PID
        pivot.setControl(pivotPositionRequest);
        /*
        setControl function sets the control mode of the motor based on the input, in this case we're using a PositionVoltage and a VoltageOut
        you could also just make a new object in the set control function but this is easier to read imo
        */
    }

    public void runPivotVoltage(double voltage) {
        pivot.setControl(new VoltageOut(voltage));
    }

    public void runRollerVoltage(double voltage) {
        VoltageOut rollerVoltageOut = new VoltageOut(voltage);
        // VoltageOut is a control setting that simply sets a motor's voltage output

        roller.setControl(rollerVoltageOut);
    }
}
