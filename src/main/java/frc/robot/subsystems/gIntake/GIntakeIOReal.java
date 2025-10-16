// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gIntake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.IntakeConstants;

public class GIntakeIOReal implements GIntakeIO {

    private PearadoxTalonFX pivot;
    private TalonFXConfiguration talonFXConfigs;

    private PearadoxTalonFX roller;

    /** Creates a new GIntakeIOReal. */
    public GIntakeIOReal() {
        pivot = new PearadoxTalonFX(
            IntakeConstants.PIVOT_ID,
            NeutralModeValue.Brake,
            IntakeConstants.PIVOT_CURRENT_LIMIT,
            false
        );
        
        talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;
        
        slot0Configs.kP = 0.1;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.1;
        
        pivot.getConfigurator().apply(slot0Configs);
        
        
        roller = new PearadoxTalonFX(
            IntakeConstants.ROLLER_ID,
            NeutralModeValue.Coast, 
            IntakeConstants.ROLLER_CURRENT_LIMIT,
            false
        );
    }

    // no getInstance function because instances are dependent on whether SIM or REAL - handled in RobotContainer

    public void updateInputs(GIntakeIOInputsAutoLogged inputs) {
        inputs.positionRots = pivot.getPosition().getValueAsDouble();
        inputs.rollerSpeedRps = roller.getVelocity().getValueAsDouble();
        
        inputs.pivotStatorCurrent = pivot.getStatorCurrent().getValueAsDouble();
        inputs.pivotSupplyCurrent = pivot.getSupplyCurrent().getValueAsDouble();
    }

    public void runPosition(double setpoint, boolean isIntaking, double feedforward) {
        PositionVoltage pivotPositionRequest = new PositionVoltage(setpoint);
        // PositionVoltage is a control setting that sets a motor's desired position and applies voltage based on PID
        // and FF to best get to that position

        VoltageOut rollerVoltageOut =
                new VoltageOut(isIntaking ? IntakeConstants.ROLLER_INTAKE_SPEED : IntakeConstants.ROLLER_OUTAKE_SPEED);
        // VoltageOut is a control setting that simply sets a motor's voltage output

        pivot.setControl(pivotPositionRequest);
        roller.setControl(rollerVoltageOut);
        /*
        setControl function sets the control mode of the motor based on the input, in this case we're using a PositionVoltage and a VoltageOut
        you could also just make a new object in the set control function but this is easier to read imo
        */
    }
}
