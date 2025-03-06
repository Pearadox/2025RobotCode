// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private PearadoxTalonFX pivot;

    private ArmMode armMode = ArmMode.Stowed;

    private enum ArmMode {
        Intake,
        L2,
        L3,
        L4,
        Stowed,
        ALGAE_LOW,
        ALGAE_HIGH,
        BARGE,
        Unpowered
    }

    private double armAdjust = 0.0;

    public static final Arm arm = new Arm();

    public static Arm getInstance() {
        return arm;
    }

    public Arm() {
        pivot = new PearadoxTalonFX(
                ArmConstants.ARM_KRAKEN_ID, NeutralModeValue.Brake, ArmConstants.CURRENT_LIMIT, true);

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

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kG = ArmConstants.kG; // add enough Gravity Gain just before motor starts moving
        slot0Configs.kS = ArmConstants.kS; // Add x output to overcome static friction
        slot0Configs.kV = ArmConstants.kV; // A velocity target of 1 rps results in x output
        slot0Configs.kA = ArmConstants.kA; // An acceleration of 1 rps/s requires x output
        slot0Configs.kP = ArmConstants.kP; // A position error of x rotations results in 12 V output
        slot0Configs.kI = ArmConstants.kI; // no output for integrated error
        slot0Configs.kD = ArmConstants.kD; // A velocity error of 1 rps results in x output

        pivot.getConfigurator().apply(slot0Configs);

        // zeroArm();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/Raw Position", pivot.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm/Angle degrees", getArmAngleDegrees());
        SmartDashboard.putNumber("Arm/Velocity rot/sec", pivot.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Arm/Voltage", pivot.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Arm/Supply Current", pivot.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Arm/Stator Current", pivot.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Arm/Adjust", armAdjust);
        SmartDashboard.putNumber(
                "Arm/Intake Setpoint", ArmConstants.ARM_INTAKE_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust);
        SmartDashboard.putNumber(
                "Arm/L4 Setpoint", ArmConstants.ARM_LEVEL_4_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust);
        SmartDashboard.putNumber(
                "Arm/Stow Setpoint", ArmConstants.ARM_STOWED_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust);
        SmartDashboard.putString("Arm/Mode", armMode.toString());
    }

    public void armHold() {
        if (armMode == ArmMode.Unpowered) {
            pivot.set(0);
            return;
        }

        double setpoint;
        if (armMode == ArmMode.Intake) {
            setpoint = ArmConstants.ARM_INTAKE_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        } else if (armMode == ArmMode.L2) {
            setpoint = ArmConstants.ARM_LEVEL_2_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        } else if (armMode == ArmMode.L3) {
            setpoint = ArmConstants.ARM_LEVEL_3_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        } else if (armMode == ArmMode.L4) {
            setpoint = ArmConstants.ARM_LEVEL_4_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        } else if (armMode == ArmMode.ALGAE_LOW) {
            setpoint = ArmConstants.ARM_ALGAE_LOW * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        } else if (armMode == ArmMode.ALGAE_HIGH) {
            setpoint = ArmConstants.ARM_ALGAE_HIGH * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        } else if (armMode == ArmMode.BARGE) {
            setpoint = ArmConstants.ARM_BARGE * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        } else {
            setpoint = ArmConstants.ARM_STOWED_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        }
        pivot.setControl(new PositionVoltage(setpoint));
        SmartDashboard.putNumber("Arm/Cur Setpoint", setpoint);
    }

    public void setArmIntake() {
        armMode = ArmMode.Intake;
    }

    public void setArmL2() {
        armMode = ArmMode.L2;
    }

    public void setArmL3() {
        armMode = ArmMode.L3;
    }

    public void setArmL4() {
        armMode = ArmMode.L4;
    }

    public void setAlgaeLow() {
        armMode = ArmMode.ALGAE_LOW;
    }

    public void setAlgaeHigh() {
        armMode = ArmMode.ALGAE_HIGH;
    }

    public void setBarge() {
        armMode = ArmMode.BARGE;
    }

    public void setStowed() {
        armMode = ArmMode.Stowed;
    }

    public void setUnpowered() {
        armMode = ArmMode.Unpowered;
    }

    public double getPivotPosition() {
        return pivot.getPosition().getValueAsDouble() / ArmConstants.ARM_GEAR_RATIO;
    }

    public double getArmAngleDegrees() {
        return Units.rotationsToDegrees(getPivotPosition());
    }

    // public void zeroArm() {
    //   pivot.setPosition(ArmConstants.ARM_STOWED_ROT * ArmConstants.ARM_GEAR_RATIO);
    // }

    public void armAdjust(double adjustBy) {
        armAdjust += adjustBy;
    }
}
