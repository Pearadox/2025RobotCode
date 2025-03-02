package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorCommands {

    public static Command setElevatorStowedMode(ElevatorSubsystem elevator) {
        return Commands.run(
                () -> {
                    elevator.setElevatorStowedMode();
                },
                elevator);
    }

    public static Command setElevatorStationMode(ElevatorSubsystem elevator) {
        return Commands.run(
                () -> {
                    elevator.setElevatorStationMode();
                },
                elevator);
    }

    public static Command setElevatorLevelTwoMode(ElevatorSubsystem elevator) {
        return Commands.run(
                () -> {
                    elevator.setElevatorLevelTwoMode();
                },
                elevator);
    }

    public static Command setElevatorLevelThreeMode(ElevatorSubsystem elevator) {
        return Commands.run(
                () -> {
                    elevator.setElevatorLevelThreeMode();
                },
                elevator);
    }

    public static Command setElevatorLevelFourMode(ElevatorSubsystem elevator) {
        return Commands.run(
                () -> {
                    elevator.setElevatorLevelFourMode();
                },
                elevator);
    }
}
