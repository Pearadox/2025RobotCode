// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.RobotContainer;

public class EndEffector extends SubsystemBase {

    private static final EndEffector END_EFFECTOR = new EndEffector();

    public static EndEffector getInstance() {
        return END_EFFECTOR;
    }

    private PearadoxTalonFX endEffector;
    private DigitalInput endSensor;
    private Debouncer debouncer;

    private boolean rumbled = false;
    private boolean isIntaking = false;
    private boolean isCoral = true; // TODO: integrate with arm
    private boolean isHolding = false;

    public EndEffector() {
        endEffector = new PearadoxTalonFX(EndEffectorConstants.END_EFFECTOR_ID, NeutralModeValue.Brake, 60, false);
        endSensor = new DigitalInput(EndEffectorConstants.END_SENSOR_CHANNEL);
        debouncer = new Debouncer(0.075, DebounceType.kFalling);

        BaseStatusSignal.setUpdateFrequencyForAll(
                ArmConstants.UPDATE_FREQ,
                endEffector.getPosition(),
                endEffector.getVelocity(),
                endEffector.getDutyCycle(),
                endEffector.getMotorVoltage(),
                endEffector.getTorqueCurrent(),
                endEffector.getSupplyCurrent(),
                endEffector.getStatorCurrent());
        SmartDashboard.putNumber("EE Speed", isCoral ? -0.15 : 0.1);
    }

    @Override
    public void periodic() {
        collectCoral();

        SmartDashboard.putBoolean("End Sensor", isHolding);
        SmartDashboard.putBoolean("Has Coral", hasCoral());

        SmartDashboard.putNumber(
                "EE Stator Current", endEffector.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber(
                "EE Supply Current", endEffector.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("EE Volts", endEffector.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(
                "EE Angular Velocity", endEffector.getVelocity().getValueAsDouble());
    }

    public void collectCoral() {
        if (RobotContainer.opController.getRightTriggerAxis() >= 0.25) {
            coralIn();
            isHolding = false;
        } else if (RobotContainer.opController.getLeftTriggerAxis()
                >= 0.25) { // warning - this left trigger is being used for ground intake too - oops
            coralOut();
            isHolding = false;
        } else if (hasCoral() && isCoral) {
            stop();
        } else if (!isHolding) {
            holdCoral();
        }
    }

    public void coralIn() {
        endEffector.set(EndEffectorConstants.PULL_SPEED);
    }

    public void coralOut() {
        endEffector.set(EndEffectorConstants.PUSH_SPEED);
    }

    public void holdCoral() {
        endEffector.set(SmartDashboard.getNumber("EE Speed", isCoral ? -0.15 : 0.1));
    }

    public void stop() {
        endEffector.set(0);
        isHolding = true;
    }

    public boolean hasCoral() {
        return debouncer.calculate(endEffector.getStatorCurrent().getValueAsDouble() > 20);
    }

    public boolean getHolding() {
        return isHolding;
    }

    public void setHolding(boolean hold) {
        isHolding = hold;
    }
}
