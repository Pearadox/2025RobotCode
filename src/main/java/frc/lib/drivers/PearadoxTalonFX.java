// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class PearadoxTalonFX extends TalonFX {
    /**
     * Creates a new TalonFX with the necessary configurations.
     *
     * @param deviceId The device ID.
     * @param mode The neutral mode (Brake/Coast).
     * @param limit The current limit.
     * @param isInverted The invert type of the motor.
     */
    public PearadoxTalonFX(int deviceId, NeutralModeValue mode, int limit, boolean isInverted) {
        this(deviceId, mode, limit, limit, isInverted);
    }

    /**
     * Creates a new TalonFX with the necessary configurations.
     *
     * @param deviceId The device ID.
     * @param mode The neutral mode (Brake/Coast).
     * @param stator The stator current limit.
     * @param supply The supply current limit.
     * @param isInverted The invert type of the motor.
     */
    public PearadoxTalonFX(int deviceId, NeutralModeValue mode, int supply, int stator, boolean isInverted) {
        super(deviceId);
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = mode;
        config.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = supply;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = stator;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        // config.CurrentLimits.SupplyCurrentLowerLimit = limit / 2; // prevent breaker trips
        // config.CurrentLimits.SupplyCurrentLowerTime = 1;

        this.getConfigurator().apply(config);
    }

    public void setCurrentLimit(double limit) {
        setCurrentLimits(limit, limit);
    }

    public void setCurrentLimits(double supply, double stator) {
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.withSupplyCurrentLimitEnable(true);
        currentLimitsConfigs.withSupplyCurrentLimit(supply);
        currentLimitsConfigs.withStatorCurrentLimitEnable(true);
        currentLimitsConfigs.withStatorCurrentLimit(stator);

        this.getConfigurator().apply(currentLimitsConfigs);
    }
}
