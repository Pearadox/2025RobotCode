package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LEDStrip;

public class AstrosLED extends Command {
    private LEDStrip ledStrip;

    public AstrosLED(LEDStrip ledStrip) {
        this.ledStrip = ledStrip;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
