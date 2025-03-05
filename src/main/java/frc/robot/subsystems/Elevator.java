// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.util.PearadoxTalonFX;

public class Elevator extends SubsystemBase {

    private PearadoxTalonFX elevator;
    private PearadoxTalonFX elevatorFollower;
    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private double elevatorOffset = 0.0;
    private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    private static enum ElevatorMode {
        STOWED,
        STATION,
        LEVEL_TWO,
        LEVEL_THREE,
        LEVEL_FOUR;
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
        slot0Configs.kG = ElevatorConstants.kG;
        slot0Configs.kS = ElevatorConstants.kS;
        slot0Configs.kV = ElevatorConstants.kV;
        slot0Configs.kA = ElevatorConstants.kA;
        slot0Configs.kP = ElevatorConstants.kP;
        slot0Configs.kI = ElevatorConstants.kI;
        slot0Configs.kD = ElevatorConstants.kD;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MM_CRUISE_VELCOCITY;
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MM_ACCELERATION;

        elevator.getConfigurator().apply(talonFXConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
                ElevatorConstants.UPDATE_FREQ,
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

        SmartDashboard.putNumber("Elevator kG", ElevatorConstants.kG);
        SmartDashboard.putNumber("Elevator kS", ElevatorConstants.kS);
        SmartDashboard.putNumber("Elevator kV", ElevatorConstants.kV);
        SmartDashboard.putNumber("Elevator kA", ElevatorConstants.kA);
        SmartDashboard.putNumber("Elevator kP", ElevatorConstants.kP);
        SmartDashboard.putNumber("Elevator kI", ElevatorConstants.kI);
        SmartDashboard.putNumber("Elevator kD", ElevatorConstants.kD);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/Position Inches", getElevatorPositionInches());
        SmartDashboard.putNumber("Elevator/Velocity (in/sec)", getElevatorVelocityInchesPerSecond());
        SmartDashboard.putNumber("Elevator/Position Rots", getElevatorPositionRots());
        SmartDashboard.putNumber("Elevator/Velocity (rot/sec)", getElevatorVelocity_RotsPerSecond());
        SmartDashboard.putNumber(
                "Elevator/Supply Current A", elevator.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(
                "Elevator/Stator Current A", elevator.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber(
                "Elevator/Voltage V", elevator.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Offset", elevatorOffset);

        setElevatorPosition();
    }

    public void setElevatorPosition() {
        double setpoint;
        if (elevatorMode == ElevatorMode.STATION) {
            setpoint = ElevatorConstants.STATION_ROT + elevatorOffset;
        } else if (elevatorMode == ElevatorMode.LEVEL_TWO) {
            setpoint = ElevatorConstants.LEVEL_TWO_ROT + elevatorOffset;
        } else if (elevatorMode == ElevatorMode.LEVEL_THREE) {
            setpoint = ElevatorConstants.LEVEL_THREE_ROT + elevatorOffset;
        } else if (elevatorMode == ElevatorMode.LEVEL_FOUR) {
            setpoint = ElevatorConstants.LEVEL_FOUR_ROT + elevatorOffset;
        } else { // stowed
            setpoint = Math.max(lowest_rot, Math.min((ElevatorConstants.STOWED_ROT + elevatorOffset), highest_rot));
        }

        elevator.setControl(motionMagicRequest.withPosition(setpoint));
        SmartDashboard.putNumber("Elevator/Setpoint", setpoint);
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

        elevator.getConfigurator().refresh(talonFXConfigs);
        elevator.getConfigurator().apply(talonFXConfigs);
    }
}
