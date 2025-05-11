// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.SmarterDashboard;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private double elevatorOffset = 0.0;

    private boolean isCoral = true;
    private boolean isAligning = false;
    private boolean isZeroing = false;

    public static enum ElevatorMode {
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

    private ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    /** Creates a new Elevator. */
    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        MechVisualizer.getInstance().updateElevatorHeight(getElevatorPositionMeters());

        SmarterDashboard.putNumber("Elevator/Offset", elevatorOffset);
        SmarterDashboard.putString("Elevator Mode", elevatorMode.toString());
        SmarterDashboard.putBoolean("Elevator/IsCoral", isCoral);
        SmarterDashboard.putNumber("Elevator/Position Inches", getElevatorPositionInches());
        SmarterDashboard.putNumber("Elevator/Velocity (in/sec)", getElevatorVelocityInchesPerSecond());
        SmarterDashboard.putNumber("Elevator/Position Rots", getElevatorPositionRots());
        SmarterDashboard.putNumber("Elevator/Velocity (rot/sec)", getElevatorVelocity_RotsPerSecond());

        // setAligning(!RobotContainer.align.isAligned());
    }

    public void setElevatorPosition() {
        double setpoint = ElevatorConstants.STOWED_ROT + elevatorOffset;

        if (elevatorMode == ElevatorMode.STOWED) {
            setpoint = ElevatorConstants.STOWED_ROT + elevatorOffset;
        } else if (elevatorMode == ElevatorMode.STATION) {
            if (isAligning) {
                setpoint = ElevatorConstants.OBSTRUCTED_STATION_ROT + elevatorOffset;
            } else {
                setpoint = ElevatorConstants.STATION_ROT + elevatorOffset;
            }
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
                    if (DriverStation.isAutonomous()) {
                        setpoint = ElevatorConstants.LEVEL_FOUR_ROT + elevatorOffset - 0.4;
                    } else {
                        setpoint = ElevatorConstants.LEVEL_FOUR_ROT + elevatorOffset;
                    }
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

        if (!isZeroing) {
            io.reachGoal(setpoint);
        } else {
            homeElevator();
        }

        SmarterDashboard.putNumber("Elevator/Setpoint", setpoint);
        // Logger.recordOutput("Elevator/Align Setpoint", RobotContainer.align.getElevatorHeightRots() +
        // elevatorOffset);
    }

    public void homeElevator() {
        io.setSpeed(0);
    }

    public void zeroElevator() {
        SmarterDashboard.putNumber("Elevator/New Zero Diff", 0 - inputs.positionRots);
        io.setPosition(0);
        elevatorOffset = 0;
    }

    public void stopElevator() {
        io.setSpeed(0);
    }

    public double getElevatorPositionRots() {
        return inputs.positionRots;
    }

    public double getElevatorVelocity_RotsPerSecond() {
        return inputs.velocityRps;
    }

    public double getElevatorPositionInches() {
        return getElevatorPositionRots() * ElevatorConstants.kRotationToInches;
    }

    public double getElevatorPositionMeters() {
        return Units.inchesToMeters(getElevatorPositionInches());
    }

    public double getElevatorVelocityInchesPerSecond() {
        return getElevatorVelocity_RotsPerSecond() * ElevatorConstants.kRotationToInches;
    }

    public ElevatorMode getElevatorMode() {
        return elevatorMode;
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
}
