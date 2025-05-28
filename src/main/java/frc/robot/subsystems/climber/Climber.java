package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.elevator.MechVisualizer;
import frc.robot.util.SmarterDashboard;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    @AutoLogOutput
    private double climberOffset = 0;

    @AutoLogOutput
    private int climbState = 1;

    @AutoLogOutput
    private String climbStateString = "unpowered";

    private ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        updateClimStateString();

        SmarterDashboard.putNumber("Climber/State", climbState);
        SmarterDashboard.putString("Climber/StateStr", climbStateString);

        MechVisualizer.getInstance().updateClimberRoll(getAngleRads());
    }

    public void climberAdjustUp() {
        climberOffset += 10; // in rotations
    }

    public void climberAdjustDown() {
        climberOffset -= 10;
    }

    public void zeroClimber() {
        io.zeroPosition();
        climberOffset = 0;
    }

    public void stop() {
        io.setSpeed(0);
    }

    public void updateClimStateString() {
        if (climbState == 2) {
            climbStateString = "Climbing!";
        } else if (climbState == 1) {
            climbStateString = "Deployed (Vertical)";
        } else if (climbState == 0) {
            climbStateString = "Retracted (Stowed)";
        }
    }

    public void setClimbPosition() {
        if (climbState == 2) {
            climbClimber();
        } else if (climbState == 1) {
            deployClimber();
        } else if (climbState == 0) {
            retractClimber();
        }
    }

    public void retractClimber() {
        io.runPosition(-290 + climberOffset);
    }

    public void climberDown() {
        io.setSpeed(-0.8);
    }

    public void climberUp() {
        io.setSpeed(0.8);
    }

    public void deployClimber() {
        io.runPosition(0 + climberOffset);
    }

    public void climbClimber() {
        io.runPosition(-195 + climberOffset);
    }

    public void incrementClimbState() {
        climbState = Math.min(2, climbState + 1);
        setClimbPosition();
    }

    public void decrementClimbState() {
        climbState = Math.max(0, climbState - 1);
        setClimbPosition();
    }

    public double getAngleRads() {
        return Units.rotationsToRadians(inputs.positionRots / ClimbConstants.GEAR_RATIO)
                + ClimbConstants.STARTING_ANGLE;
    }
}
