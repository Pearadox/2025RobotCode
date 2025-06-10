// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;
import frc.robot.util.SmarterDashboard;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
    private Debouncer debouncer;

    private boolean isCoral = true;
    private boolean isHoldingCoral = true;
    private boolean isHoldingAlgae = false;
    private boolean holdSpeed = false;

    private double lastRot = 0;

    private EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    public EndEffector(EndEffectorIO io) {
        this.io = io;

        debouncer = new Debouncer(0.125, DebounceType.kRising);

        SmarterDashboard.putNumber("EE/EE Speed", isCoral ? -0.15 : 0.1);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EE", inputs);

        collectGamePiece();

        SmarterDashboard.putBoolean("EE/Holding Coral", isHoldingCoral);
        SmarterDashboard.putBoolean("EE/Holding Algae", isHoldingAlgae);
        SmarterDashboard.putBoolean("EE/Has Coral", hasCoral());
        SmarterDashboard.putBoolean("EE/Holding Speed", holdSpeed);
    }

    public void collectGamePiece() {
        if (DriverStation.isTeleop()) {
            if (RobotContainer.driverController.getLeftBumperButton()) {
                if (isCoral) {
                    coralIn();
                    holdSpeed = false;
                } else if (!isCoral) {
                    algaeIn();
                    holdSpeed = true;
                }
                isHoldingCoral = false;
            } else if (RobotContainer.driverController.getRightBumperButton()) {
                if (isCoral) {
                    coralOut();
                } else if (!isCoral) {
                    algaeOut();
                }

                holdSpeed = false;
                isHoldingCoral = false;
            } else {
                if (!holdSpeed) {
                    stop();
                }
                if (isCoral) {
                    holdCoral();
                }
            }
        }
    }

    // scores
    public void coralIn() {
        io.setSpeed(EndEffectorConstants.PULL_SPEED);
        setLastRot();
    }

    public void algaeIn() {
        io.setSpeed(EndEffectorConstants.ALGAE_PULL_SPEED);
    }

    public void algaeOut() {
        io.setSpeed(EndEffectorConstants.ALGAE_PUSH_SPEED);
    }

    // intakes
    public void coralOut() {
        if (Arm.getArmMode() == ArmMode.L2 || Arm.getArmMode() == ArmMode.Stowed) {
            io.setSpeed(EndEffectorConstants.L2_PUSH_SPEED);
        } else if (Arm.getArmMode() == ArmMode.L3) {
            io.setSpeed(EndEffectorConstants.L3_PUSH_SPEED);
        } else {
            io.setSpeed(EndEffectorConstants.PUSH_SPEED);
        }
        setLastRot();
    }

    public void holdCoral() {
        // endEffector.set(SmartDashboard.getNumber("EE/EE Speed", isCoral ? -0.15 : 0.1));
        io.setSpeed(EndEffectorConstants.HOLD_SPEED);
        setLastRot();
    }

    public void holdAlgae() {
        io.setSpeed(EndEffectorConstants.ALGAE_PULL_SPEED);
    }

    public void stopCoral() {
        stop();
        isHoldingCoral = true;
    }

    public void stopAlgae() {
        stop();
        isHoldingAlgae = true;
    }

    public void stop() {
        io.setSpeed(0);
    }

    public boolean hasCoral() {
        return debouncer.calculate(inputs.statorCurrent > 30);
    }

    public boolean getHolding() {
        return isHoldingCoral;
    }

    public void setHolding(boolean hold) {
        isHoldingCoral = hold;
    }

    public void setLastRot() {
        lastRot = inputs.positionRots;
    }

    public void setCoral() {
        isCoral = true;
    }

    public void setAlgae() {
        isCoral = false;
    }

    public boolean isCoral() {
        return isCoral;
    }
}
