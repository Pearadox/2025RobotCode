package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

    private static enum ElevatorMode {
        STOWED,
        STATION,
        LEVEL_TWO,
        LEVEL_THREE,
        LEVEL_FOUR;
    }

    private ElevatorMode elevatorMode = ElevatorMode.STOWED;

    private final ElevatorIO io;
    private final ElevatorIOInputAutoLogged inputs;

    private double goalPosition = 0;
    private double elevatorOffset = 0;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
        inputs = new ElevatorIOInputAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator/inputs", inputs);
        Logger.recordOutput("Elevator/goalPosition", goalPosition * ElevatorConstants.kRotationToInches);

        if (elevatorMode == ElevatorMode.STOWED) {
            if (elevatorMode == ElevatorMode.STOWED) {
                io.setPosition(ElevatorConstants.STOWED_ROT + elevatorOffset);
                goalPosition = ElevatorConstants.STOWED_ROT + elevatorOffset;
            } else if (elevatorMode == ElevatorMode.STATION) {
                io.setPosition(ElevatorConstants.STATION_ROT + elevatorOffset);
                goalPosition = ElevatorConstants.STATION_ROT + elevatorOffset;
            } else if (elevatorMode == ElevatorMode.LEVEL_TWO) {
                io.setPosition(ElevatorConstants.LEVEL_TWO_ROT + elevatorOffset);
                goalPosition = ElevatorConstants.LEVEL_TWO_ROT + elevatorOffset;
            } else if (elevatorMode == ElevatorMode.LEVEL_THREE) {
                io.setPosition(ElevatorConstants.LEVEL_THREE_ROT + elevatorOffset);
                goalPosition = ElevatorConstants.LEVEL_THREE_ROT + elevatorOffset;
            } else if (elevatorMode == ElevatorMode.LEVEL_FOUR) {
                io.setPosition(ElevatorConstants.LEVEL_FOUR_ROT + elevatorOffset);
                goalPosition = ElevatorConstants.LEVEL_FOUR_ROT + elevatorOffset;
            }
        }
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
}
