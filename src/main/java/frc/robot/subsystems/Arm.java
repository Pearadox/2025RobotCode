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
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;
import frc.robot.util.SmarterDashboard;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private PearadoxTalonFX pivot;
    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    private ArmMode armMode = ArmMode.Stowed;
    private TalonFXConfiguration talonFXConfigs;

    private VoltageOut voltageRequest = new VoltageOut(0);

    private boolean isCoral = true;
    private boolean isAligning = false;
    private double lastAngle = 0;

    private enum ArmMode {
        Intake,
        L2,
        L3,
        L4,
        Stowed,
        LOLLICLIMB
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

        talonFXConfigs.Voltage.PeakForwardVoltage = 3.0;
        talonFXConfigs.Voltage.PeakReverseVoltage = -3.0;

        // var slot0Configs = talonFXConfigs.Slot0;
        // slot0Configs.kG = ArmConstants.kG; // add enough Gravity Gain just before motor starts moving
        // slot0Configs.kS = ArmConstants.kS; // Add x output to overcome static friction
        // slot0Configs.kV = ArmConstants.kV; // A velocity target of 1 rps results in x output
        // slot0Configs.kA = ArmConstants.kA; // An acceleration of 1 rps/s requires x output
        // slot0Configs.kP = ArmConstants.kP; // A position error of x rotations results in 12 V output
        // slot0Configs.kI = ArmConstants.kI; // no output for integrated error
        // slot0Configs.kD = ArmConstants.kD; // A velocity error of 1 rps results in x output

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = 0; // add enough Gravity Gain just before motor starts moving
        slot0Configs.kS = 0.15; // Add x output to overcome static friction
        slot0Configs.kV = 7.2; // A velocity target of 1 rps results in x output
        slot0Configs.kA = 0; // An acceleration of 1 rps/s requires x output
        slot0Configs.kP = 0.4; // A position error of x rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity error of 1 rps results in x output
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        var slot1Configs = talonFXConfigs.Slot1; // configs for game piece
        slot1Configs.kG = 0;
        slot1Configs.kS = 0;
        slot1Configs.kV = 0;
        slot1Configs.kA = 0;
        slot1Configs.kP = 0;
        slot1Configs.kI = 0;
        slot1Configs.kD = 0;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.MM_MAX_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = ArmConstants.MM_MAX_CRUISE_ACCELERATION;

        // talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // talonFXConfigs.Feedback.FeedbackRotorOffset = -16;

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
        SmarterDashboard.putNumber("Arm/DeltaAngle", deltaArmAngle());
        SmarterDashboard.putNumber("Arm/kG", 0.35 * Math.cos(-1 * Units.degreesToRadians(getArmAngleDegrees() - 96)));

        setAligning(!RobotContainer.align.isAligned());
    }

    public void armHold() {

        // pivot.setControl(voltageRequest.withOutput(armAdjust));

        double setpoint = ArmConstants.ARM_STOWED_ROT + armAdjust;
        // if (armMode == ArmMode.Intake) {
        //     if (isCoral) {
        //         setpoint = ArmConstants.ARM_INTAKE_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        //     } else {
        //         setpoint = ArmConstants.ARM_LOLIPOP * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        //     }
        // } else if (armMode == ArmMode.L2) {
        //     if (isCoral) {
        //         setpoint = ArmConstants.ARM_LEVEL_2_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        //     } else {
        //         setpoint = ArmConstants.ARM_ALGAE_LOW * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        //     }
        // } else if (armMode == ArmMode.L3) {
        //     if (isCoral) {
        //         setpoint = ArmConstants.ARM_LEVEL_3_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        //     } else {
        //         setpoint = ArmConstants.ARM_ALGAE_HIGH * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        //     }
        // } else if (armMode == ArmMode.L4) {
        //     setpoint = ArmConstants.ARM_LEVEL_4_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust;

        // } else if (armMode == ArmMode.ALGAE_LOW) {
        //     setpoint = ArmConstants.ARM_ALGAE_LOW * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        // } else if (armMode == ArmMode.ALGAE_HIGH) {
        //     setpoint = ArmConstants.ARM_ALGAE_HIGH * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        // } else if (armMode == ArmMode.BARGE) {
        //     setpoint = ArmConstants.ARM_BARGE * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        // } else if (armMode == ArmMode.CLIMB) {
        //     setpoint = ArmConstants.ARM_CLIMB * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        // } else {
        //     setpoint = ArmConstants.ARM_STOWED_ROT * ArmConstants.ARM_GEAR_RATIO + armAdjust;
        // }
        if (isCoral) {
            if (armMode == ArmMode.Stowed) {
                setpoint = ArmConstants.ARM_STOWED_ROT + armAdjust;
            } else if (armMode == ArmMode.Intake) {
                setpoint = ArmConstants.ARM_INTAKE_ROT + armAdjust;
            } else if (armMode == ArmMode.L2) {
                setpoint = ArmConstants.ARM_LEVEL_2_ROT + armAdjust;
            } else if (armMode == ArmMode.L3) {
                setpoint = ArmConstants.ARM_LEVEL_3_ROT + armAdjust;
            } else if (armMode == ArmMode.L4) {
                if (isAligning) {
                    // setpoint = RobotContainer.align.getArmAngleRots() + armAdjust;
                    setpoint = ArmConstants.ARM_L4_BEHIND_CORAL + armAdjust;
                } else {
                    setpoint = ArmConstants.ARM_LEVEL_4_ROT + armAdjust;
                }
            }
        } else if (!isCoral) {
            if (armMode == ArmMode.Stowed) {
                setpoint = ArmConstants.ARM_LOLLIPOP + armAdjust;
            } else if (armMode == ArmMode.L2) {
                setpoint = ArmConstants.ARM_ALGAE_LOW + armAdjust;
            } else if (armMode == ArmMode.L3) {
                setpoint = ArmConstants.ARM_ALGAE_HIGH + armAdjust;
            } else if (armMode == ArmMode.L4) {
                setpoint = ArmConstants.ARM_BARGE + armAdjust;
            }
        }

        Logger.recordOutput("Arm/Align Setpoint", RobotContainer.align.getArmAngleRots() + armAdjust);

        // pivot.setControl(motionMagicRequest.withPosition(setpoint));
        pivot.setControl(new PositionVoltage(-setpoint)
                .withFeedForward(0.35 * Math.cos(-1 * Units.degreesToRadians(getArmAngleDegrees() - 96))));
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

    // public void setAlgaeLow() {
    //     armMode = ArmMode.ALGAE_LOW;
    // }

    // public void setAlgaeHigh() {
    //     armMode = ArmMode.ALGAE_HIGH;
    // }

    // public void setBarge() {
    //     armMode = ArmMode.BARGE;
    // }

    public void setStowed() {
        armMode = ArmMode.Stowed;
    }

    public void setLollipopOrClimb() {
        armMode = ArmMode.LOLLICLIMB;
    }

    // public void setUnpowered() {
    //     armMode = ArmMode.Unpowered;
    // }

    public void setCoral() {
        isCoral = true;
    }

    public void setAlgae() {
        isCoral = false;
    }

    public boolean getIsCoral() {
        return isCoral;
    }

    public void setAligning(boolean flag) {
        isAligning = flag;
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

    // public double getAutoArmAngle(){
    //     double dist = RobotContainer.align.getTagDist(); //this doesnt work maybe make AutoAlign a subsystem

    //     return Math.acos((dist + ArmConstants.ARM_LENGTH * Math.cos(0)) / ArmConstants.ARM_LENGTH); //TODO Current
    // Angle Setpoint
    // }
}
