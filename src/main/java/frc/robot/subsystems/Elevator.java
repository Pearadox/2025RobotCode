// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.SmarterDashboard;

public class Elevator extends SubsystemBase {

    private PearadoxTalonFX elevator;
    private PearadoxTalonFX elevatorFollower;
    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private double elevatorOffset = 0.0;
    private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    private boolean isCoral = true;

    private static enum ElevatorMode {
        STOWED,
        STATION,
        LEVEL_TWO,
        LEVEL_THREE,
        LEVEL_FOUR,
        ALGAE_LOW,
        ALGAE_HIGH,
        BARGE
    }

    private ElevatorMode elevatorMode = ElevatorMode.STOWED;
    private double lowest_rot = ElevatorConstants.STOWED_ROT;
    private double highest_rot = ElevatorConstants.MAX_ELEVATOR_ROT;

    private static Elevator ELEVATOR = new Elevator();

    public static Elevator getInstance() {
        return ELEVATOR;
    }
    /** Creates a new Elevator. */
    public Elevator() {
        elevator = new PearadoxTalonFX(
                ElevatorConstants.ELEVATOR_ID,
                ElevatorConstants.MODE,
                ElevatorConstants.CURRENT_LIMIT,
                ElevatorConstants.IS_INVERTED);

        elevatorFollower = new PearadoxTalonFX(
                ElevatorConstants.ELEVATOR_FOLLOWER_ID,
                ElevatorConstants.MODE,
                ElevatorConstants.CURRENT_LIMIT,
                ElevatorConstants.IS_INVERTED);

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = ElevatorConstants.kG; // add enough Gravity Gain just before motor starts moving
        slot0Configs.kS = ElevatorConstants.kS; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = ElevatorConstants.kV; // A velocity target of 1 rps results in 0.1 V output
        slot0Configs.kA = ElevatorConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = ElevatorConstants.kP; // A position error of 2.5 rotations results in 12 V output, prev 4.8
        slot0Configs.kI = ElevatorConstants.kI; // no output for integrated error
        slot0Configs.kD = ElevatorConstants.kD; // A velocity error of 1 rps results in 0.1 V output

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity =
                ElevatorConstants.MM_CRUISE_VELCOCITY; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration =
                ElevatorConstants.MM_ACCELERATION; // Target acceleration of 160 rps/s (0.5 seconds)
        // (not sure if needed - > ) motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1
        // seconds)

        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 33.5;
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        talonFXConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        elevator.getConfigurator().apply(talonFXConfigs);

        // elevator.getPosition().setUpdateFrequency(ElevatorConstants.UPDATE_FREQ);
        // elevator.getVelocity().setUpdateFrequency(ElevatorConstants.UPDATE_FREQ);

        // // These are needed for the follower motor to work
        // elevator.getDutyCycle().setUpdateFrequency(ElevatorConstants.UPDATE_FREQ);
        // elevator.getMotorVoltage().setUpdateFrequency(ElevatorConstants.UPDATE_FREQ);
        // elevator.getTorqueCurrent().setUpdateFrequency(ElevatorConstants.UPDATE_FREQ);
        // elevator.getSupplyCurrent().setUpdateFrequency(ElevatorConstants.UPDATE_FREQ);
        // elevator.getStatorCurrent().setUpdateFrequency(ElevatorConstants.UPDATE_FREQ);

        BaseStatusSignal.setUpdateFrequencyForAll(
                ArmConstants.UPDATE_FREQ,
                elevator.getPosition(),
                elevator.getVelocity(),
                elevator.getDutyCycle(),
                elevator.getMotorVoltage(),
                elevator.getTorqueCurrent(),
                elevator.getSupplyCurrent(),
                elevator.getStatorCurrent());

        elevator.optimizeBusUtilization();

        elevatorFollower.getConfigurator().apply(talonFXConfigs);
        elevatorFollower.optimizeBusUtilization();
        elevatorFollower.setControl(new Follower(ElevatorConstants.ELEVATOR_ID, true));

        SmarterDashboard.putNumber("Elevator kG", ElevatorConstants.kG);
        SmarterDashboard.putNumber("Elevator kS", ElevatorConstants.kS);
        SmarterDashboard.putNumber("Elevator kV", ElevatorConstants.kV);
        SmarterDashboard.putNumber("Elevator kA", ElevatorConstants.kA);
        SmarterDashboard.putNumber("Elevator kP", ElevatorConstants.kP);
        SmarterDashboard.putNumber("Elevator kI", ElevatorConstants.kI);
        SmarterDashboard.putNumber("Elevator kD", ElevatorConstants.kD);
    }

    @Override
    public void periodic() {
        SmarterDashboard.putNumber("Elevator/Position Inches", getElevatorPositionInches());
        SmarterDashboard.putNumber("Elevator/Velocity (in/sec)", getElevatorVelocityInchesPerSecond());
        SmarterDashboard.putNumber("Elevator/Position Rots", getElevatorPositionRots());
        SmarterDashboard.putNumber("Elevator/Velocity (rot/sec)", getElevatorVelocity_RotsPerSecond());
        SmarterDashboard.putNumber(
                "Elevator/Supply Current A", elevator.getSupplyCurrent().getValueAsDouble());
        SmarterDashboard.putNumber(
                "Elevator/Stator Current A", elevator.getStatorCurrent().getValueAsDouble());
        SmarterDashboard.putNumber(
                "Elevator/Voltage V", elevator.getMotorVoltage().getValueAsDouble());
        SmarterDashboard.putNumber("Elevator/Offset", elevatorOffset);
        SmarterDashboard.putString("Elevator Mode", elevatorMode.toString());
        SmarterDashboard.putBoolean("Elevator/IsCoral", isCoral);
    }

    public void setElevatorPosition() {
        double setpoint = ElevatorConstants.STOWED_ROT + elevatorOffset;
        if (elevatorMode == ElevatorMode.STATION) {
            if (isCoral) {
                setpoint = ElevatorConstants.STATION_ROT + elevatorOffset;
            } else {
                setpoint = ElevatorConstants.STOWED_ROT + elevatorOffset;
            }
        } else if (elevatorMode == ElevatorMode.LEVEL_TWO) {
            if (isCoral) {
                setpoint = ElevatorConstants.LEVEL_TWO_ROT + elevatorOffset;
            } else {
                setpoint = ElevatorConstants.ALGAE_LOW_ROT + elevatorOffset;
            }
        } else if (elevatorMode == ElevatorMode.LEVEL_THREE) {
            if (isCoral) {
                setpoint = ElevatorConstants.LEVEL_THREE_ROT + elevatorOffset;
            } else {
                setpoint = ElevatorConstants.ALGAE_HIGH_ROT + elevatorOffset;
            }
        } else if (elevatorMode == ElevatorMode.LEVEL_FOUR) {
            setpoint = ElevatorConstants.LEVEL_FOUR_ROT + elevatorOffset;
        } else if (elevatorMode == ElevatorMode.ALGAE_LOW) {
            setpoint = ElevatorConstants.ALGAE_LOW_HEIGHT + elevatorOffset;
        } else if (elevatorMode == ElevatorMode.ALGAE_HIGH) {
            setpoint = ElevatorConstants.ALGAE_HIGH_HEIGHT + elevatorOffset;
        } else if (elevatorMode == ElevatorMode.BARGE) {
            setpoint = ElevatorConstants.BARGE_ROT + elevatorOffset;
        }

        // } else { // stowed
        //   setpoint = Math.max(lowest_rot, Math.min((ElevatorConstants.STOWED_ROT + elevatorOffset), highest_rot));
        // }

        // switch (elevatorMode) {
        //   case LEVEL_FOUR: setpoint = ElevatorConstants.LEVEL_FOUR_ROT + elevatorOffset; break;
        //   case LEVEL_THREE: setpoint = ElevatorConstants.LEVEL_THREE_ROT + elevatorOffset; break;
        //   case LEVEL_TWO: setpoint = ElevatorConstants.LEVEL_TWO_ROT + elevatorOffset; break;
        //   case STATION: setpoint = ElevatorConstants.STATION_ROT + elevatorOffset; break;
        //   default: setpoint = Math.max(lowest_rot, Math.min((ElevatorConstants.STOWED_ROT + elevatorOffset),
        // highest_rot));
        // }

        elevator.setControl(motionMagicRequest.withPosition(setpoint));
        SmarterDashboard.putNumber("Elevator/Setpoint", setpoint);
    }

    public double getElevatorPositionRots() {
        return elevator.getPosition().getValueAsDouble();
    }

    public double getElevatorVelocity_RotsPerSecond() {
        return elevator.getVelocity().getValueAsDouble();
    }

    public double getElevatorPositionInches() {
        return getElevatorPositionRots() * ElevatorConstants.kRotationToInches;
    }

    public double getElevatorVelocityInchesPerSecond() {
        return getElevatorVelocity_RotsPerSecond() * ElevatorConstants.kRotationToInches;
    }

    public void changeElevatorOffset(double value) {
        elevatorOffset += value;
    }

    public void setElevatorStowedMode() {
        elevatorMode = ElevatorMode.STOWED;
    }

    public void setElevatorStationMode() {
        elevatorMode = ElevatorMode.STATION;
    }

    public void setElevatorLevelTwoMode() {
        elevatorMode = ElevatorMode.LEVEL_TWO;
    }

    public void setElevatorLevelThreeMode() {
        elevatorMode = ElevatorMode.LEVEL_THREE;
    }

    public void setElevatorLevelFourMode() {
        elevatorMode = ElevatorMode.LEVEL_FOUR;
    }

    public void setElevatorAlgaeLow() {
        elevatorMode = ElevatorMode.ALGAE_LOW;
    }

    public void setElevatorAlgaeHigh() {
        elevatorMode = ElevatorMode.ALGAE_HIGH;
    }

    public void setElevatorBarge() {
        elevatorMode = ElevatorMode.BARGE;
    }

    public void setCoral() {
        isCoral = true;
    }

    public void setAlgae() {
        isCoral = false;
    }

    public boolean getIsCoral() {
        return isCoral;
    }

    public void changeIsCoral() {
        isCoral = !isCoral;
    }

    public void resetAdjust() {
        elevatorOffset = 0;
    }

    public void setPID() {

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = SmartDashboard.getNumber("Elevator kG", ElevatorConstants.kG);
        slot0Configs.kS = SmartDashboard.getNumber("Elevator kS", ElevatorConstants.kS);
        slot0Configs.kV = SmartDashboard.getNumber("Elevator kV", ElevatorConstants.kV);
        slot0Configs.kA = SmartDashboard.getNumber("Elevator kA", ElevatorConstants.kA);
        slot0Configs.kP = SmartDashboard.getNumber("Elevator kP", ElevatorConstants.kP);
        slot0Configs.kI = SmartDashboard.getNumber("Elevator kI", ElevatorConstants.kI);
        slot0Configs.kD = SmartDashboard.getNumber("Elevator kD", ElevatorConstants.kD);

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity =
                ElevatorConstants.MM_CRUISE_VELCOCITY; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration =
                ElevatorConstants.MM_ACCELERATION; // Target acceleration of 160 rps/s (0.5 seconds)
        // (not sure if needed - > ) motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1
        // seconds)

        elevator.getConfigurator().refresh(talonFXConfigs);
        elevator.getConfigurator().apply(talonFXConfigs);
    }
}
