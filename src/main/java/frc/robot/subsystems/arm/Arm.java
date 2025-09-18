// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.subsystems.elevator.MechVisualizer;
import frc.robot.util.RobotIdentity;
import frc.robot.util.SmarterDashboard;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    // private InverseKinematics ik = new InverseKinematics();

    private boolean isCoral = true;
    private boolean isAligning = false;
    private double lastAngle = 0.0;
    private double armAdjust = Constants.IDENTITY == RobotIdentity.EVE ? -0.71 : 0.0;

    public enum ArmMode {
        Intake,
        L2,
        L3,
        L4,
        Stowed,
        LOLLICLIMB
    }

    private static ArmMode armMode = ArmMode.Stowed;

    private ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public Arm(ArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        MechVisualizer.getInstance().updateArmAngle(getArmAngleRadsToHorizontal());

        SmarterDashboard.putNumber("Arm/Angle degrees", getArmAngleDegrees());
        SmarterDashboard.putNumber("Arm/Adjust", armAdjust);
        SmarterDashboard.putString("Arm/Mode", armMode.toString());
        SmarterDashboard.putBoolean("Arm/IsCoral", isCoral);
        SmarterDashboard.putNumber("Arm/DeltaAngle", deltaArmAngle());
        SmarterDashboard.putNumber("Arm/kG", 0.35 * Math.cos(-1 * Units.degreesToRadians(getArmAngleDegrees() - 96)));

        // setAligning(!RobotContainer.align.isAligned());
    }

    public void armHold() {
        double setpoint = ArmConstants.ARM_STOWED_ROT + armAdjust;

        if (isCoral) {
            if (armMode == ArmMode.Stowed) {
                setpoint = ArmConstants.ARM_STOWED_ROT + armAdjust;
            } else if (armMode == ArmMode.Intake) {
                if (isAligning) {
                    setpoint = ArmConstants.ARM_STATION_BEHIND_CORAL + armAdjust;

                } else {
                    setpoint = ArmConstants.ARM_INTAKE_ROT + armAdjust;
                }
            } else if (armMode == ArmMode.L2) {
                // setpoint = ik.getArmAngleRots(AlignConstants.REEF_ALIGN_TZ, AlignConstants.L2_HEIGHT) + armAdjust;
                setpoint = ArmConstants.ARM_LEVEL_2_ROT + armAdjust;
            } else if (armMode == ArmMode.L3) {
                // setpoint = ik.getArmAngleRots(AlignConstants.REEF_ALIGN_TZ, AlignConstants.L3_HEIGHT) + armAdjust -
                // 2.5;
                setpoint = ArmConstants.ARM_LEVEL_3_ROT + armAdjust;

                if (Constants.currentMode == Constants.Mode.SIM) {
                    setpoint += 1;
                }
                if (Constants.IDENTITY == RobotIdentity.EVE) {
                    setpoint += 2; // 12 degrees
                }
            } else if (armMode == ArmMode.L4) {
                // setpoint = ik.getArmAngleRots(AlignConstants.REEF_ALIGN_TZ, AlignConstants.L4_HEIGHT) + armAdjust;
                setpoint = ArmConstants.ARM_LEVEL_4_ROT + armAdjust;
            }
        } else if (!isCoral) {
            if (armMode == ArmMode.Stowed) {
                setpoint = ArmConstants.ARM_LOLLIPOP + armAdjust;
            } else if (armMode == ArmMode.L2) {
                setpoint = ArmConstants.ARM_ALGAE_LOW + armAdjust;
            } else if (armMode == ArmMode.L3) {
                setpoint = ArmConstants.ARM_ALGAE_HIGH + armAdjust;
            } else if (armMode == ArmMode.L4) {
                setpoint = ArmConstants.ARM_BARGE + armAdjust;
            }
        }

        // Logger.recordOutput("Arm/Align Setpoint", RobotContainer.align.getArmAngleRots() + armAdjust);

        // pivot.setControl(motionMagicRequest.withPosition(setpoint));
        io.runPosition(-setpoint, getFeedforwardVolts());

        // pivot.setControl(new PositionVoltage(-setpoint).withFeedForward(ArmConstants.kG *
        // Math.cos(getArmAngleDegrees() - 96)));
        SmarterDashboard.putNumber("Arm/Cur Setpoint", -setpoint);
    }

    public double deltaArmAngle() {
        var temp = lastAngle;
        lastAngle = getArmAngleDegrees();
        return lastAngle - temp;
    }

    public void setArmIntake() {
        armMode = ArmMode.Intake;
    }

    public void setArmL2() {
        armMode = ArmMode.L2;
    }

    public void setArmL3() {
        armMode = ArmMode.L3;
    }

    public void setArmL4() {
        armMode = ArmMode.L4;
    }

    public void setStowed() {
        armMode = ArmMode.Stowed;
    }

    public void setLollipopOrClimb() {
        armMode = ArmMode.LOLLICLIMB;
    }

    public void setCoral() {
        isCoral = true;
    }

    public void setAlgae() {
        isCoral = false;
    }

    public static ArmMode getArmMode() {
        return armMode;
    }

    public boolean getIsCoral() {
        return isCoral;
    }

    public void setAligning(boolean flag) {
        isAligning = flag;
    }

    public double getPivotPosition() {
        return inputs.positionRots / ArmConstants.ARM_GEAR_RATIO;
    }

    public double getArmAngleDegrees() {
        return Units.rotationsToDegrees(getPivotPosition());
    }

    public double getArmAngleRadsToHorizontal() {
        return Units.rotationsToRadians(inputs.positionRots / ArmConstants.ARM_GEAR_RATIO)
                + SimulationConstants.STARTING_ANGLE;
    }

    public double getFeedforwardVolts() {
        return 0.35 * Math.cos(-1 * Units.degreesToRadians(getArmAngleDegrees() - 96));
    }

    public void armAdjust(double adjustBy) {
        armAdjust += adjustBy;
    }

    public void resetAdjust() {
        armAdjust = 0;
    }
}
