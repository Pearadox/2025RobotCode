// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;
import frc.robot.util.SmarterDashboard;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
    private boolean isCoralMode = true;
    private boolean isHoldingCoral = true;
    private boolean isHoldingAlgae = false;
    private boolean needsHoldSpeed = false;

    private EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    public EndEffector(EndEffectorIO io) {
        this.io = io;

        SmarterDashboard.putNumber("EE/EE Speed", isCoralMode ? -0.15 : 0.1);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        inputs.hasCoral = true;

        Logger.processInputs("EE", inputs);

        collectGamePiece();

        SmarterDashboard.putBoolean("EE/Holding Coral", isHoldingCoral);
        SmarterDashboard.putBoolean("EE/Holding Algae", isHoldingAlgae);
        SmarterDashboard.putBoolean("EE/Has Coral", hasCoral());
        SmarterDashboard.putBoolean("EE/Holding Speed", needsHoldSpeed);
    }

    public void collectGamePiece() {
        if (DriverStation.isAutonomous()) return;

        boolean isIntaking = RobotContainer.driverController.getLeftBumperButton();
        boolean isOuttaking = RobotContainer.driverController.getRightBumperButton();

        if (isIntaking) {
            if (isCoralMode) {
                intakeCoral();
                needsHoldSpeed = false;
            } else {
                intakeAlgae();
                needsHoldSpeed = true;
            }
            isHoldingCoral = false;
            return;
        }

        if (isOuttaking) {
            if (isCoralMode) {
                outtakeCoral();
            } else {
                outtakeAlgae();
            }
            needsHoldSpeed = false;
            isHoldingCoral = false;
            return;
        }

        // Idle behavior
        if (!needsHoldSpeed) stopEE();
        if (isCoralMode) holdCoral();
    }

    public void intakeCoral() {
        io.setSpeed(EndEffectorConstants.PULL_SPEED);
        setLastRot();
    }

    public void intakeAlgae() {
        io.setSpeed(EndEffectorConstants.ALGAE_PULL_SPEED);
    }

    public void outtakeAlgae() {
        io.setSpeed(EndEffectorConstants.ALGAE_PUSH_SPEED);
    }

    public void outtakeCoral() {
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
        stopEE();
        isHoldingCoral = true;
    }

    public void stopAlgae() {
        stopEE();
        isHoldingAlgae = true;
    }

    public void stopEE() {
        io.setSpeed(0);
    }

    public boolean hasCoral() {
        return inputs.hasCoral;
    }

    public boolean getHolding() {
        return isHoldingCoral;
    }

    public void setHolding(boolean hold) {
        isHoldingCoral = hold;
    }

    public void setLastRot() {}

    public void setCoralMode() {
        isCoralMode = true;
    }

    public void setAlgaeMode() {
        isCoralMode = false;
    }

    public boolean getIsCoralMode() {
        return isCoralMode;
    }
}
