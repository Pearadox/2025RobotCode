// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gIntake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.gIntake.GIntakeConstants.GIntakeState;

public class GIntake extends SubsystemBase {

    private GIntakeState gIntakeState = GIntakeState.STOWED;

    private double setpoint;

    private GIntakeIO io;

    private GIntakeIOInputsAutoLogged inputs = new GIntakeIOInputsAutoLogged();

    /** Creates a new GIntake. */
    public GIntake(GIntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() { // mostly for logging, but generally for all things that need to happen periodically :O
        // This method will be called once per scheduler run
        io.updateInputs(inputs);

        io.runPosition(gIntakeState.getPivotSetpoint(), gIntakeState.getRollerSpeed());
    }

    public GIntakeState getState() {
        return gIntakeState;
    }
    
    public double getPivotRawPosition() {
        return inputs.positionRots / GIntakeConstants.PIVOT_GEARING; // divide by gearing bc the pivot's position is not exact to the motor's position
    }
    
    public double getPivotAngleDegrees() {
        return Units.rotationsToDegrees(getPivotRawPosition());
    }

    // vvv state modifiers vvv

    public void setStowed() {
        gIntakeState = GIntakeState.STOWED;
    }

    public void setIntake() {
        gIntakeState = GIntakeState.INTAKE;
    }

    public void setOuttake() {
        gIntakeState = GIntakeState.OUTTAKE;
    }

    public void setAlgae() {
        gIntakeState = GIntakeState.ALGAE;
    }

    public void adjustSetpoint(double rotations) {
        gIntakeState.adjustSetpoint(rotations);
    }
}
