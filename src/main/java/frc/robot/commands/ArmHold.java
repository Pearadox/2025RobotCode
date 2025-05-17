package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmHold extends Command {

    private Arm arm = Arm.getInstance();

    public ArmHold() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        arm.armHold();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
