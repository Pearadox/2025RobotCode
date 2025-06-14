package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;
import org.littletonrobotics.junction.Logger;

public class InverseKinematics {
    public InverseKinematics() {
        // nothing
    }

    private double getElevatorHeightMeters(double branchX, double branchY) {
        double elevatorHeightMeters = branchY
                - Math.sqrt(Math.pow(AlignConstants.PIVOT_TO_CORAL_RADIUS, 2)
                        - Math.pow(branchX + AlignConstants.BRANCH_OFFSET_BEHIND_APRILTAG, 2));

        if (Double.isNaN(elevatorHeightMeters)) {
            return Double.NaN;
        }

        return elevatorHeightMeters;
    }

    public double getElevatorHeightRots(double branchX, double branchY) {
        double elevatorHeightMeters = getElevatorHeightMeters(branchX, branchY);
        Logger.recordOutput("Elevator/AlignElevatorHeight", elevatorHeightMeters);
        Logger.recordOutput("Elevator/AlignElevatorIsNaN", Double.isNaN(elevatorHeightMeters));
        double rots = 0;

        if (Double.isNaN(elevatorHeightMeters)) {
            return ElevatorConstants.LEVEL_FOUR_ROT;
        }

        if (Arm.getArmMode() == ArmMode.L2) {
            elevatorHeightMeters = elevatorHeightMeters
                    + 2 * AlignConstants.PIVOT_TO_CORAL_RADIUS * Math.sin(getArmAngleRads(branchX, branchY));
        }

        rots = Units.metersToInches(elevatorHeightMeters - AlignConstants.ELEVATOR_STARTING_HEIGHT)
                * ElevatorConstants.GEAR_RATIO
                / (Math.PI * ElevatorConstants.PULLEY_DIAMETER);

        return rots;
    }

    private double getArmAngleRads(double branchX, double branchY) {
        double armAngle = Math.acos(
                (branchX + AlignConstants.BRANCH_OFFSET_BEHIND_APRILTAG) / AlignConstants.PIVOT_TO_CORAL_RADIUS);

        if (Double.isNaN(armAngle)) {
            return Double.NaN;
        }

        return armAngle;
    }

    public double getArmAngleRots(double branchX, double branchY) {
        double armAngleRads = getArmAngleRads(branchX, branchY);
        Logger.recordOutput(
                "Arm/AlignDegreesFromHorizontal",
                Units.radiansToDegrees(armAngleRads + AlignConstants.ARM_TO_CORAL_ANGULAR_OFFSET));
        Logger.recordOutput("Arm/AlignIsNaN", Double.isNaN(armAngleRads));
        double rots = 0;

        if (Double.isNaN(armAngleRads)) {
            return ArmConstants.ARM_LEVEL_4_ROT;
        }

        if (Arm.getArmMode() == ArmMode.L2) {
            armAngleRads = armAngleRads - 2 * armAngleRads;
        }

        rots = Units.radiansToRotations((armAngleRads
                        + (-1 * AlignConstants.ARM_STARTING_ANGLE)
                        + AlignConstants.ARM_TO_CORAL_ANGULAR_OFFSET))
                * ArmConstants.ARM_GEAR_RATIO;

        return rots;
    }

    // public double getTZForArmSpacing(Arm arm) {
    //     ArmMode armMode = arm.getArmMode();

    //     switch (armMode) {
    //         case L2:
    //             return AlignConstants.SPACING_TZ;
    //         case L3:
    //             return 0.0;
    //         case L4:
    //             return 0.0;
    //         default:
    //             return 0.0;
    //     }
    // }
}
