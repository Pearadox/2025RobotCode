// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.util.SmarterDashboard;

public class GroundIntake extends SubsystemBase {
    private PearadoxTalonFX pivot;
    private PearadoxTalonFX roller;

    PositionVoltage pivotRequest = new PositionVoltage(0).withSlot(0);

    public enum PivotPos {
        stowed,
        intake,
        outtake,
        algae
    }

    private boolean isCoral = true;

    public PivotPos pivotPos = PivotPos.stowed;

    private static final GroundIntake GROUND_INTAKE = new GroundIntake();

    public static GroundIntake getInstance() {
        return GROUND_INTAKE;
    }

    public GroundIntake() {
        pivot = new PearadoxTalonFX(
                GroundIntakeConstants.PIVOT_ID,
                GroundIntakeConstants.MODE,
                GroundIntakeConstants.PIVOT_CURRENT_LIMIT,
                false);

        Slot0Configs slot0configs = new Slot0Configs();

        slot0configs.kP = GroundIntakeConstants.PIVOT_kP;
        slot0configs.kI = GroundIntakeConstants.PIVOT_kI;
        slot0configs.kD = GroundIntakeConstants.PIVOT_kD;

        pivot.getConfigurator().apply(slot0configs);

        roller = new PearadoxTalonFX(
                GroundIntakeConstants.ROLLER_ID,
                GroundIntakeConstants.MODE,
                GroundIntakeConstants.ROLLER_CURRENT_LIMIT,
                false);
    }

    public void setStowed() {
        pivotPos = PivotPos.stowed;
    }

    public void setIntakePos() {
        pivotPos = PivotPos.intake;
    }

    public void setOuttakePos() {
        pivotPos = PivotPos.outtake;
    }

    public void setAlgaePos() {
        pivotPos = PivotPos.algae;
    }

    public PivotPos getPivotPos() {
        return pivotPos;
    }

    public void intake() {
        roller.set(GroundIntakeConstants.PULL_SPEED);
    }

    public void Outtake() {
        roller.set(-GroundIntakeConstants.PUSH_SPEED);
    }

    public void pivotHold() {
        if (isCoral) {
            if (pivotPos == PivotPos.stowed) {
                pivot.setControl(pivotRequest.withPosition(GroundIntakeConstants.PIVOT_STOWED_ROT));
                roller.set(0);
            } else if (pivotPos == PivotPos.intake) {
                pivot.setControl(pivotRequest.withPosition(GroundIntakeConstants.PIVOT_INTAKE_ROT));
                if (DriverStation.isTeleop() && RobotContainer.opController.getLeftTriggerAxis() > 0.6) {
                    roller.set(GroundIntakeConstants.PULL_SPEED);
                }
            } else if (pivotPos == PivotPos.outtake) {
                pivot.setControl(pivotRequest.withPosition(GroundIntakeConstants.PIVOT_OUTTAKE_ROT));
                if (DriverStation.isTeleop() && RobotContainer.opController.getRightTriggerAxis() > 0.6) {
                    roller.set(-GroundIntakeConstants.PUSH_SPEED);
                }
            }
        } else if (!isCoral) {
            if (pivotPos == PivotPos.stowed) {
                pivot.setControl(pivotRequest.withPosition(GroundIntakeConstants.PIVOT_STOWED_ROT));
                roller.set(0);
            } else if (pivotPos == PivotPos.intake) {
                pivot.setControl(pivotRequest.withPosition(GroundIntakeConstants.PIVOT_ALGAE_ROT));
                if (DriverStation.isTeleop() && RobotContainer.opController.getLeftTriggerAxis() > 0.6) {
                    roller.set(GroundIntakeConstants.ALGAE_PULL_SPEED);
                }
            } else if (pivotPos == PivotPos.outtake) {
                pivot.setControl(pivotRequest.withPosition(GroundIntakeConstants.PIVOT_ALGAE_ROT));
                if (DriverStation.isTeleop() && RobotContainer.opController.getRightTriggerAxis() > 0.6) {
                    roller.set(-GroundIntakeConstants.ALGAE_PUSH_SPEED);
                }
            }
        }
    }

    public void setCoralMode() {
        isCoral = true;
    }

    public void setAlgaeMode() {
        isCoral = false;
    }

    public boolean getIsCoralMode() {
        return isCoral;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
                "Ground Intake Supply Curent", pivot.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(
                "Ground Intake Stator Curent", pivot.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Ground Intake position", pivot.getPosition().getValueAsDouble());
        SmarterDashboard.putBoolean("Ground Intake/IsCoral", isCoral);
    }
}
