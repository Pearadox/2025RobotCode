// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gIntake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.MechVisualizer;
import frc.robot.subsystems.gIntake.GIntakeConstants.GIntakeState;
import frc.robot.util.SmarterDashboard;
import org.littletonrobotics.junction.Logger;

public class GIntake extends SubsystemBase {

    private GIntakeState gIntakeState = GIntakeState.CORAL_STOWED;

    private boolean isCoral = true;

    private double appliedVoltage = 0.0;

    private GIntakeIO io;

    private final GIntakeIOInputsAutoLogged inputs = new GIntakeIOInputsAutoLogged();

    /** Creates a new GIntake. */
    public GIntake(GIntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() { // mostly for logging, but generally for all things that need to happen periodically :O
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("GIntake", inputs);

        io.runPivotPosition(gIntakeState.getPivotSetpoint());
        io.runRollerVoltage(gIntakeState.getRollerSpeed());

        // io.runPivotVoltage(appliedVoltage);

        SmarterDashboard.putString("GIntake/State", gIntakeState.toString());
        SmarterDashboard.putNumber("GIntake/Angle ROTS", Units.radiansToRotations(getAngleRadsFromStarting()));
        SmarterDashboard.putNumber(
                "GIntake/setpoint ROTS", gIntakeState.getPivotSetpoint() / GIntakeConstants.PIVOT_GEARING);
        SmarterDashboard.putNumber("GIntake/Adjust ROTS", gIntakeState.getAdjust() / GIntakeConstants.PIVOT_GEARING);
        SmarterDashboard.putNumber("GIntake/Roller pos", inputs.rollerPositionRots);
        SmarterDashboard.putNumber("GIntake/Applied voltage", appliedVoltage);

        MechVisualizer.getInstance()
                .updateGIntakePivotAngle(getAngleRadsFromStarting() + GIntakeConstants.PIVOT_STARTING_ANGLE);
        MechVisualizer.getInstance().updateGIntakeRollerAngle(inputs.rollerPositionRots);
    }

    public GIntakeState getState() {
        return gIntakeState;
    }

    public double getAngleRadsFromStarting() {
        return Units.rotationsToRadians(inputs.pivotPositionRots / GIntakeConstants.PIVOT_GEARING);
        // divide by gearing bc the pivot's position is not exact to the motor's position
    }

    public double getArmAngleDegreesFromStarting() {
        return Units.radiansToDegrees(getAngleRadsFromStarting());
    }

    // vvv state modifiers vvv

    public void setStowed() {
        gIntakeState = (isCoral ? GIntakeState.CORAL_STOWED : GIntakeState.ALGAE_STOWED);
    }

    public void setIntake() {
        gIntakeState = (isCoral ? GIntakeState.CORAL_INTAKE : GIntakeState.ALGAE_INTAKE);
    }

    public void setOuttake() {
        gIntakeState = (isCoral ? GIntakeState.CORAL_OUTTAKE : GIntakeState.ALGAE_OUTTAKE);
    }

    public void setCoral() {
        isCoral = true;
    }

    public void setAlgae() {
        isCoral = false;
    }

    public void adjustSetpoint(double rotations) {
        gIntakeState.adjustSetpoint(rotations);
    }

    public void resetAdjust() {
        gIntakeState.resetAdjust();
    }

    public void incrementAppliedVoltage(double by) {
        appliedVoltage += by;
    }
}
