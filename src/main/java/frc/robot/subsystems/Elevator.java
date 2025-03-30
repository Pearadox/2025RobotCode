// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotContainer;
import frc.robot.util.SmarterDashboard;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private PearadoxTalonFX elevator;
    private PearadoxTalonFX elevatorFollower;
    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private double elevatorOffset = 0.0;
    private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    private boolean isCoral = true;
    private boolean isAligning = false;
    private boolean isZeroing = false;

    private static enum ElevatorMode {
        STOWED,
        STATION,
        LEVEL_TWO,
        LEVEL_THREE,
        LEVEL_FOUR,
        ALGAE_LOW,
        ALGAE_HIGH,
        BARGE,
    }

    private ElevatorMode elevatorMode = ElevatorMode.STOWED;
    // private double lowest_rot = ElevatorConstants.STOWED_ROT;
    // private double highest_rot = ElevatorConstants.MAX_ELEVATOR_ROT;

    private boolean isLowering = false;

    private static Elevator ELEVATOR = new Elevator();

    public static Elevator getInstance() {
        return ELEVATOR;
    }

    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> elevator.setControl(m_voltReq.withOutput(volts.in(Volts))), null, this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTranslation.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTranslation.dynamic(direction);
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

        var slot1Configs = talonFXConfigs.Slot1;
        slot1Configs.kG = ElevatorConstants.kG; // add enough Gravity Gain just before motor starts moving
        slot1Configs.kS = ElevatorConstants.kS; // Add 0.1 V output to overcome static friction
        slot1Configs.kV = ElevatorConstants.kV * 0.25; // A velocity target of 1 rps results in 0.1 V output
        slot1Configs.kA = ElevatorConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot1Configs.kP = ElevatorConstants.kP; // A position error of 2.5 rotations results in 12 V output, prev 4.8
        slot1Configs.kI = ElevatorConstants.kI; // no output for integrated error
        slot1Configs.kD = ElevatorConstants.kD; // A velocity error of 1 rps results in 0.1 V output

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity =
                ElevatorConstants.MM_CRUISE_VELCOCITY_UP; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration =
                ElevatorConstants.MM_ACCELERATION_UP; // Target acceleration of 160 rps/s (0.5 seconds)
        // (not sure if needed - > ) motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1
        // seconds)

        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                33 * ElevatorConstants.GEAR_RATIO / (Math.PI * ElevatorConstants.PULLEY_DIAMETER);
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

        setAligning(!RobotContainer.align.isAligned());
    }

    public void setElevatorPosition() {
        double setpoint = ElevatorConstants.STOWED_ROT + elevatorOffset;
        // if (elevatorMode == ElevatorMode.STATION) {
        //     if (isCoral) {
        //         setpoint = ElevatorConstants.STATION_ROT + elevatorOffset;
        //     } else {
        //         setpoint = ElevatorConstants.STOWED_ROT + elevatorOffset;
        //     }
        // } else if (elevatorMode == ElevatorMode.LEVEL_TWO) {
        //     if (isCoral) {
        //         setpoint = ElevatorConstants.LEVEL_TWO_ROT + elevatorOffset;
        //     } else {
        //         setpoint = ElevatorConstants.ALGAE_LOW_ROT + elevatorOffset;
        //     }
        // } else if (elevatorMode == ElevatorMode.LEVEL_THREE) {
        //     if (isCoral) {
        //         setpoint = ElevatorConstants.LEVEL_THREE_ROT + elevatorOffset;
        //     } else {
        //         setpoint = ElevatorConstants.ALGAE_HIGH_ROT + elevatorOffset;
        //     }
        // } else if (elevatorMode == ElevatorMode.LEVEL_FOUR) {
        //     setpoint = ElevatorConstants.LEVEL_FOUR_ROT + elevatorOffset;
        // } else if (elevatorMode == ElevatorMode.ALGAE_LOW) {
        //     setpoint = ElevatorConstants.ALGAE_LOW_HEIGHT + elevatorOffset;
        // } else if (elevatorMode == ElevatorMode.ALGAE_HIGH) {
        //     setpoint = ElevatorConstants.ALGAE_HIGH_HEIGHT + elevatorOffset;
        // } else if (elevatorMode == ElevatorMode.BARGE) {
        //     setpoint = ElevatorConstants.BARGE_ROT + elevatorOffset;
        // }

        if (elevatorMode == ElevatorMode.STOWED) {
            setpoint = ElevatorConstants.STOWED_ROT + elevatorOffset;
        } else if (elevatorMode == ElevatorMode.STATION) {
            setpoint = ElevatorConstants.STATION_ROT + elevatorOffset;
        }
        if (isCoral) {
            if (elevatorMode == ElevatorMode.LEVEL_TWO) {
                setpoint = ElevatorConstants.LEVEL_TWO_ROT + elevatorOffset;
            } else if (elevatorMode == ElevatorMode.LEVEL_THREE) {
                setpoint = ElevatorConstants.LEVEL_THREE_ROT + elevatorOffset;
            } else if (elevatorMode == ElevatorMode.LEVEL_FOUR) {
                if (isAligning) {
                    // setpoint = RobotContainer.align.getElevatorHeightRots() + elevatorOffset;
                    // setpoint = ElevatorConstants.BARGE_ROT + elevatorOffset;
                    setpoint = ElevatorConstants.LEVEL_FOUR_ROT + elevatorOffset;

                } else {
                    setpoint = ElevatorConstants.LEVEL_FOUR_ROT + elevatorOffset;
                }
            }
        } else if (!isCoral) {
            if (elevatorMode == ElevatorMode.LEVEL_TWO) {
                setpoint = ElevatorConstants.ALGAE_LOW_ROT + elevatorOffset;
            } else if (elevatorMode == ElevatorMode.LEVEL_THREE) {
                setpoint = ElevatorConstants.ALGAE_HIGH_ROT + elevatorOffset;
            } else if (elevatorMode == ElevatorMode.LEVEL_FOUR) {
                setpoint = ElevatorConstants.BARGE_ROT + elevatorOffset;
            }
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

        isLowering = elevator.getPosition().getValueAsDouble() > setpoint;

        if (!isZeroing) {
            elevator.setControl(motionMagicRequest.withPosition(setpoint).withSlot(isLowering ? 1 : 0));
        } else {
            homeElevator();
        }

        // // only reapply configs when this changes
        // if (isLowering != (elevator.getPosition().getValueAsDouble() > setpoint)) {
        //     isLowering = elevator.getPosition().getValueAsDouble() > setpoint;

        //     if (isLowering) {
        //         talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MM_CRUISE_VELCOCITY_DOWN;
        //         talonFXConfigs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MM_ACCELERATION_DOWN;
        //     } else {
        //         talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MM_CRUISE_VELCOCITY_UP;
        //         talonFXConfigs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MM_ACCELERATION_UP;
        //     }
 
        //     elevator.getConfigurator().refresh(talonFXConfigs);
        //     elevator.getConfigurator().apply(talonFXConfigs);

        //     SmarterDashboard.putBoolean("Elevator/IsLowering", isLowering);
        // }
        SmarterDashboard.putBoolean("Elevator/IsLowering", isLowering);

        SmarterDashboard.putNumber("Elevator/Setpoint", setpoint);
        Logger.recordOutput("Elevator/Align Setpoint", RobotContainer.align.getElevatorHeightRots() + elevatorOffset);
    }

    public void homeElevator() {
        elevator.set(ElevatorConstants.HOMING_SPEED);
    }

    public void zeroElevator() {
        SmarterDashboard.putNumber(
                "Elevator/New Zero Diff", 0 - elevator.getPosition().getValueAsDouble());
        elevator.setPosition(0);
        elevatorOffset = 0;
    }

    public void stopElevator() {
        elevator.set(0);
    }

    // public double getAutoHeightAdjust(){
    //     double angle = RobotContainer.arm.getAutoArmAngle();

    //     return
    // }

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

    public void setAligning(boolean flag) {
        isAligning = flag;
    }

    public void setZeroing(boolean flag) {
        isZeroing = flag;
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
                ElevatorConstants.MM_CRUISE_VELCOCITY_UP; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration =
                ElevatorConstants.MM_ACCELERATION_UP; // Target acceleration of 160 rps/s (0.5 seconds)
        // (not sure if needed - > ) motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1
        // seconds)

        elevator.getConfigurator().refresh(talonFXConfigs);
        elevator.getConfigurator().apply(talonFXConfigs);
    }
}
