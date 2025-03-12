// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.SmarterDashboard;

public class Arm extends SubsystemBase {
    private PearadoxTalonFX pivot;
    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    private ArmMode armMode = ArmMode.Stowed;
    private TalonFXConfiguration talonFXConfigs;

    private boolean isCoral = true;
    private double lastAngle = 0;

    private enum ArmMode {
        Intake,
        L2,
        L3,
        L4,
        Stowed,
        ALGAE_LOW,
        ALGAE_HIGH,
        BARGE,
        CLIMB,
        LOLIPOP,
        Unpowered
    }

    private double armAdjust = 0.0;

    public static final Arm arm = new Arm();

    public static Arm getInstance() {
        return arm;
    }

    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> pivot.setControl(m_voltReq.withOutput(volts.in(Volts))), null, this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTranslation.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTranslation.dynamic(direction);
    }

    public Arm() {
        pivot = new PearadoxTalonFX(
                ArmConstants.ARM_KRAKEN_ID, NeutralModeValue.Brake, ArmConstants.CURRENT_LIMIT, false);

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

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = ArmConstants.kG; // add enough Gravity Gain just before motor starts moving
        slot0Configs.kS = ArmConstants.kS; // Add x output to overcome static friction
        slot0Configs.kV = ArmConstants.kV; // A velocity target of 1 rps results in x output
        slot0Configs.kA = ArmConstants.kA; // An acceleration of 1 rps/s requires x output
        slot0Configs.kP = ArmConstants.kP; // A position error of x rotations results in 12 V output
        slot0Configs.kI = ArmConstants.kI; // no output for integrated error
        slot0Configs.kD = ArmConstants.kD; // A velocity error of 1 rps results in x output

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.MM_MAX_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = ArmConstants.MM_MAX_CRUISE_ACCELERATION;

        pivot.getConfigurator().apply(talonFXConfigs);
    }

    @Override
    public void periodic() {
        SmarterDashboard.putNumber("Arm/Raw Position", pivot.getPosition().getValueAsDouble());
        SmarterDashboard.putNumber("Arm/Angle degrees", getArmAngleDegrees());
        SmarterDashboard.putNumber("Arm/Velocity rot/sec", pivot.getVelocity().getValueAsDouble());
        SmarterDashboard.putNumber("Arm/Voltage", pivot.getMotorVoltage().getValueAsDouble());
        SmarterDashboard.putNumber(
                "Arm/Supply Current", pivot.getSupplyCurrent().getValueAsDouble());
        SmarterDashboard.putNumber(
                "Arm/Stator Current", pivot.getStatorCurrent().getValueAsDouble());
        SmarterDashboard.putNumber("Arm/Adjust", armAdjust);
        SmarterDashboard.putNumber(
                "Arm/Intake Setpoint", ArmConstants.ARM_INTAKE_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust);
        SmarterDashboard.putNumber(
                "Arm/L4 Setpoint", ArmConstants.ARM_LEVEL_4_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust);
        SmarterDashboard.putNumber(
                "Arm/Stow Setpoint", ArmConstants.ARM_STOWED_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust);
        SmarterDashboard.putString("Arm/Mode", armMode.toString());
        SmarterDashboard.putBoolean("Arm/IsCoral", isCoral);
        deltaArmAngle();
        SmarterDashboard.putNumber("Arm/DeltaAngle", deltaArmAngle());
    }

    public void armHold() {
        talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = ArmConstants.kG
                * Math.cos(getArmAngleDegrees() - 96); // add enough Gravity Gain just before motor starts moving
        slot0Configs.kS = ArmConstants.kS; // Add x output to overcome static friction
        slot0Configs.kV = ArmConstants.kV; // A velocity target of 1 rps results in x output
        slot0Configs.kA = ArmConstants.kA; // An acceleration of 1 rps/s requires x output
        slot0Configs.kP = ArmConstants.kP; // A position error of x rotations results in 12 V output
        slot0Configs.kI = ArmConstants.kI; // no output for integrated error
        slot0Configs.kD = ArmConstants.kD; // A velocity error of 1 rps results in x output

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.MM_MAX_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = ArmConstants.MM_MAX_CRUISE_ACCELERATION;

        double setpoint;
        if (armMode == ArmMode.Intake) {
            if (isCoral) {
                setpoint = ArmConstants.ARM_INTAKE_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust;
            } else {
                setpoint = ArmConstants.ARM_LOLIPOP * ArmConstants.ARM_GEAR_RATIO + armAdjust;
            }
        } else if (armMode == ArmMode.L2) {
            if (isCoral) {
                setpoint = ArmConstants.ARM_LEVEL_2_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust;
            } else {
                setpoint = ArmConstants.ARM_ALGAE_LOW * ArmConstants.ARM_GEAR_RATIO + armAdjust;
            }
        } else if (armMode == ArmMode.L3) {
            if (isCoral) {
                setpoint = ArmConstants.ARM_LEVEL_3_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust;
            } else {
                setpoint = ArmConstants.ARM_ALGAE_HIGH * ArmConstants.ARM_GEAR_RATIO + armAdjust;
            }
        } else if (armMode == ArmMode.L4) {
            setpoint = ArmConstants.ARM_LEVEL_4_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust;

        } else if (armMode == ArmMode.ALGAE_LOW) {
            setpoint = ArmConstants.ARM_ALGAE_LOW * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        } else if (armMode == ArmMode.ALGAE_HIGH) {
            setpoint = ArmConstants.ARM_ALGAE_HIGH * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        } else if (armMode == ArmMode.BARGE) {
            setpoint = ArmConstants.ARM_BARGE * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        } else if (armMode == ArmMode.CLIMB) {
            setpoint = ArmConstants.ARM_CLIMB * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        } else {
            setpoint = ArmConstants.ARM_STOWED_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        }

        // pivot.setControl(motionMagicRequest.withPosition(setpoint));
        pivot.setControl(new PositionVoltage(-setpoint));
        // pivot.setControl(new PositionVoltage(-setpoint).withFeedForward(ArmConstants.kG *
        // Math.cos(getArmAngleDegrees() - 96)));
        SmarterDashboard.putNumber("Arm/Cur Setpoint", -setpoint);
    }

    public double deltaArmAngle() {
        var temp = lastAngle;
        lastAngle = getArmAngleDegrees();
        return lastAngle - temp;
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

    public void setClimb() {
        armMode = ArmMode.CLIMB;
    }

    public void setUnpowered() {
        armMode = ArmMode.Unpowered;
    }

    public void setCoral() {
        isCoral = true;
    }

    public void setAlgae() {
        isCoral = false;
    }

    public void changeIsCoral() {
        isCoral = !isCoral;
    }

    public boolean getIsCoral() {
        return isCoral;
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

    public void resetAdjust() {
        armAdjust = 0;
    }
}
