package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;

public class ElevatorCommands {

    public static Command setElevatorStowedMode(Elevator elevator) {
        return Commands.run(
                () -> {
                    elevator.setElevatorStowedMode();
                },
                elevator);
    }

    public static Command setElevatorStationMode(Elevator elevator) {
        return Commands.run(
                () -> {
                    elevator.setElevatorStationMode();
                },
                elevator);
    }

    public static Command setElevatorLevelTwoMode(Elevator elevator) {
        return Commands.run(
                () -> {
                    elevator.setElevatorLevelTwoMode();
                },
                elevator);
    }

    public static Command setElevatorLevelThreeMode(Elevator elevator) {
        return Commands.run(
                () -> {
                    elevator.setElevatorLevelThreeMode();
                },
                elevator);
    }

    public static Command setElevatorLevelFourMode(Elevator elevator) {
        return Commands.run(
                () -> {
                    elevator.setElevatorLevelFourMode();
                },
                elevator);
    }
}
