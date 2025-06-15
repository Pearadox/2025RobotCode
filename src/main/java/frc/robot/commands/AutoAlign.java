package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoAlign {
    @Setter
    private Supplier<Pose2d> poseSupplier;

    private final PIDController translationController =
            new PIDController(AlignConstants.REEF_kP, AlignConstants.REEF_kI, AlignConstants.REEF_kD);
    private final PIDController rotationController =
            new PIDController(AlignConstants.ROT_REEF_kP, AlignConstants.ROT_REEF_kI, AlignConstants.ROT_REEF_kD);

    @AutoLogOutput
    private double distanceError = 0;

    @AutoLogOutput
    private Pose2d targetPose = new Pose2d();

    private Pose2d currentPose = new Pose2d();
    private final Map<Integer, Pose3d> tagPoses3d = loadTagPositions();

    private int[] tagIDs = {};

    @AutoLogOutput
    private double currentBranchTx = AlignConstants.REEF_ALIGN_MID_TX;

    @AutoLogOutput
    private double currentBranchTz = AlignConstants.REEF_ALIGN_TZ;

    @AutoLogOutput
    @Getter
    private double xVelocity = 0;

    @AutoLogOutput
    @Getter
    private double yVelocity = 0;

    @AutoLogOutput
    @Getter
    private double angularVelocity = 0;

    @AutoLogOutput
    private Translation2d translationOutput = new Translation2d();

    @AutoLogOutput
    private double rotationOutput = 0;

    @AutoLogOutput
    @Getter
    private boolean aligningLeft = false;

    public AutoAlign(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(3));
        translationController.setTolerance(Units.inchesToMeters(4));
    }

    private Pose2d getTagPose(int tagID) {
        if (tagPoses3d.containsKey(tagID)) {
            Pose3d tagPose = tagPoses3d.get(tagID);
            return tagPose.toPose2d();
        } else return new Pose2d();
    }

    private int findClosestTag(int[] tagIDs, Pose2d curPose) {
        double minDistance = Double.POSITIVE_INFINITY;
        int closestTag = tagIDs[0];

        for (int tagID : tagIDs) {
            Pose3d tagPose = tagPoses3d.get(tagID);
            if (tagPose == null) continue;
            double distance = curPose.getTranslation()
                    .getDistance(tagPose.getTranslation().toTranslation2d());

            if (distance < minDistance) {
                minDistance = distance;
                closestTag = tagID;
            }
        }
        return closestTag;
    }

    public void updateFieldRelativeAlignSpeeds() {
        currentPose = poseSupplier.get();
        int currentTagID = findClosestTag(tagIDs, currentPose);
        targetPose = getTagPose(currentTagID)
                .transformBy(new Transform2d(new Translation2d(currentBranchTz, currentBranchTx), new Rotation2d()));

        updateControllerOutputs();
        updateVelocities();

        Logger.recordOutput("AutoAlign/RotationErrorRad", rotationController.getError());
        Logger.recordOutput("AutoAlign/RotationErrorDeg", Math.toDegrees(rotationController.getError()));
        Logger.recordOutput("AutoAlign/CurrentBranchTag", currentTagID);
    }

    public void updateControllerOutputs() {
        Translation2d currentToTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
        distanceError = currentToTarget.getNorm();
        Rotation2d directionToTarget = currentToTarget.getAngle();
        if (RobotContainer.isRedAlliance()) directionToTarget = directionToTarget.plus(Rotation2d.k180deg);

        double translationMagnitude = translationController.calculate(0, distanceError);
        translationOutput = new Translation2d(translationMagnitude, directionToTarget);

        rotationOutput = rotationController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    }

    public void updateVelocities() {
        if (isAligned()) {
            xVelocity = 0;
            yVelocity = 0;
            angularVelocity = 0;
        } else {
            xVelocity = translationOutput.getX();
            yVelocity = translationOutput.getY();
            angularVelocity = rotationOutput;

            xVelocity += 0.1 * Math.signum(xVelocity);
            yVelocity += 0.1 * Math.signum(yVelocity);
            angularVelocity += 0.1 * Math.signum(angularVelocity);
        }
    }

    public void setTagIDs(boolean isReef) {
        if (RobotContainer.isRedAlliance()) {
            tagIDs = isReef ? FieldConstants.RED_REEF_TAG_IDS : FieldConstants.RED_CORAL_STATION_TAG_IDS;
        } else {
            tagIDs = isReef ? FieldConstants.BLUE_REEF_TAG_IDS : FieldConstants.BLUE_CORAL_STATION_TAG_IDS;
        }
    }

    public void setBranchTx(double tx) {
        currentBranchTx = tx;
    }

    @AutoLogOutput(key = "AutoAlign/isAligned")
    public boolean isAligned() {
        return translationController.atSetpoint() && rotationController.atSetpoint();
    }

    public void reset() {
        translationController.reset();
        rotationController.reset();
    }

    private static Map<Integer, Pose3d> loadTagPositions() {
        Map<Integer, Pose3d> tagMap = new HashMap<>();
        for (int tag : FieldConstants.RED_REEF_TAG_IDS) {
            tagMap.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        for (int tag : FieldConstants.BLUE_REEF_TAG_IDS) {
            tagMap.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        return tagMap;
    }

    private Command getAlignCommand(Drive drive, boolean isReef, double tx) {
        return new InstantCommand(() -> {
                    setTagIDs(isReef);
                    setBranchTx(tx);
                })
                .andThen(DriveCommands.joystickDrive(
                                drive, () -> getXVelocity(), () -> getYVelocity(), () -> getAngularVelocity(), true)
                        .alongWith(new RunCommand(() -> updateFieldRelativeAlignSpeeds())));
    }

    public Command reefAlignLeft(Drive drive) {
        // aligningLeft = true;
        return getAlignCommand(drive, true, AlignConstants.REEF_ALIGN_LEFT_TX);
    }

    public Command reefAlignMid(Drive drive) {
        return getAlignCommand(drive, true, AlignConstants.REEF_ALIGN_MID_TX);
    }

    public Command reefAlignRight(Drive drive) {
        // aligningLeft = false;
        return getAlignCommand(drive, true, AlignConstants.REEF_ALIGN_RIGHT_TX);
    }

    public Command stationAlign(Drive drive) {
        return getAlignCommand(drive, false, AlignConstants.STATION_ALIGN_TX);
    }


    // Dynamic IK
    private double getElevatorHeightMeters(double branchY) {
        currentPose = poseSupplier.get();
        targetPose = getTagPose(findClosestTag(tagIDs, currentPose))
            .transformBy(
                new Transform2d(new Translation2d(
                    -1 * AlignConstants.BRANCH_OFFSET_BEHIND_APRILTAG, isAligningLeft() ? AlignConstants.REEF_ALIGN_LEFT_TX : AlignConstants.REEF_ALIGN_RIGHT_TX),
                new Rotation2d()));

        double distance = targetPose.getTranslation().minus(currentPose.getTranslation()).getNorm();

        double elevatorHeightMeters = branchY
                - Math.sqrt(Math.pow(AlignConstants.PIVOT_TO_CORAL_RADIUS, 2) - Math.pow(distance, 2));

        if (Double.isNaN(elevatorHeightMeters)) {
            return Double.NaN;
        }

        return elevatorHeightMeters;
    }

    public double getElevatorHeightRots(double branchY) {
        double elevatorHeightMeters = getElevatorHeightMeters(branchY);
        Logger.recordOutput("Elevator/AlignElevatorHeight", elevatorHeightMeters);
        Logger.recordOutput("Elevator/AlignElevatorIsNaN", Double.isNaN(elevatorHeightMeters));
        double rots = 0;

        if (Double.isNaN(elevatorHeightMeters)) {
            return ElevatorConstants.LEVEL_THREE_ROT;
        }

        if (Arm.getArmMode() == ArmMode.L2) {
            elevatorHeightMeters = elevatorHeightMeters
                    + 2 * AlignConstants.PIVOT_TO_CORAL_RADIUS * Math.sin(getArmAngleRads(branchY));
        }

        rots = Units.metersToInches(elevatorHeightMeters - AlignConstants.ELEVATOR_STARTING_HEIGHT)
                * ElevatorConstants.GEAR_RATIO
                / (Math.PI * ElevatorConstants.PULLEY_DIAMETER);

        return rots;
    }

    private double getArmAngleRads(double branchY) {
        currentPose = poseSupplier.get();
        targetPose = getTagPose(findClosestTag(tagIDs, currentPose))
            .transformBy(
                new Transform2d(new Translation2d(
                    -1 * AlignConstants.BRANCH_OFFSET_BEHIND_APRILTAG, isAligningLeft() ? AlignConstants.REEF_ALIGN_LEFT_TX : AlignConstants.REEF_ALIGN_RIGHT_TX),
                new Rotation2d()));

        double distance = targetPose.getTranslation().minus(currentPose.getTranslation()).getNorm();

        double armAngle = Math.acos(
                (distance) / AlignConstants.PIVOT_TO_CORAL_RADIUS);

        if (Double.isNaN(armAngle)) {
            return Double.NaN;
        }

        return armAngle;
    }

    public double getArmAngleRots(double branchY) {
        double armAngleRads = getArmAngleRads(branchY);
        Logger.recordOutput(
                "Arm/AlignDegreesFromHorizontal",
                Units.radiansToDegrees(armAngleRads + AlignConstants.ARM_TO_CORAL_ANGULAR_OFFSET));
        Logger.recordOutput("Arm/AlignIsNaN", Double.isNaN(armAngleRads));
        double rots = 0;

        if (Double.isNaN(armAngleRads)) {
            return ArmConstants.ARM_ALGAE_LOW;
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
}
