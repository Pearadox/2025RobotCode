// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundintake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.IntakeConstants;

public class GroundIntake extends SubsystemBase {
    /** Creates a new GroundIntake. */
    private PearadoxTalonFX pivot;

    PositionVoltage pivotRequest = new PositionVoltage(0).withSlot(0);

    // private double pivotAdjust = 0.0;

    public enum PivotPos {
        stowed(0, 0), // TODO: adjust?
        intake(-105, 0.2),
        outtake(-10, -0.2),
        algae(-40, 0.2);
        public final double pivotRots, rollerSpeed;

        private PivotPos(double pivotDegs, double rollerSpeed) {
            this.pivotRots = Units.degreesToRotations(pivotDegs) * IntakeConstants.PIVOT_GEARING;
            this.rollerSpeed = rollerSpeed;
        }
    }

    public PivotPos pivotPos = PivotPos.stowed;

    private PearadoxTalonFX roller;

    public GroundIntake() {
        pivot = new PearadoxTalonFX(
                IntakeConstants.PIVOT_ID, IntakeConstants.MODE, IntakeConstants.PIVOT_CURRENT_LIMIT, false);

        Slot0Configs slot0configs = new Slot0Configs();

        slot0configs.kP = IntakeConstants.PIVOT_kP;
        slot0configs.kI = IntakeConstants.PIVOT_kI;
        slot0configs.kD = IntakeConstants.PIVOT_kD;

        pivot.getConfigurator().apply(slot0configs);

        roller = new PearadoxTalonFX(
                IntakeConstants.ROLLER_ID, IntakeConstants.MODE, IntakeConstants.ROLLER_CURRENT_LIMIT, false);
    }

    public void setState(PivotPos state) {
        pivotPos = state;
    }

    public void changePivotActivePos() {
        if (pivotPos == PivotPos.intake) {
            pivotPos = PivotPos.outtake;
        } else if (pivotPos == PivotPos.outtake) {
            pivotPos = PivotPos.intake;
        }
    }

    public PivotPos getPivotPos() {
        return pivotPos;
    }

    public void pivotHold() {
        pivot.setControl(pivotRequest.withPosition(pivotPos.pivotRots));
        roller.set(pivotPos.rollerSpeed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        pivotHold();

        SmartDashboard.putNumber(
                "Ground Intake Supply Cuurent", pivot.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(
                "Ground Intake Stator Cuurent", pivot.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Ground Intake position", pivot.getPosition().getValueAsDouble());
    }
}
