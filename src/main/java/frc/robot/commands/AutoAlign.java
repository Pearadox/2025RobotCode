package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
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
    private boolean isReef = true;

    @AutoLogOutput
    @Getter
    private double yVelocity = 0;

    @AutoLogOutput
    @Getter
    private double angularVelocity = 0;

    @AutoLogOutput
    private Translation2d translationOutput = new Translation2d();

    @AutoLogOutput
    private String lastAlignCommand = "";

    @AutoLogOutput
    private String currentAlignCommand = "";

    @AutoLogOutput
    private double rotationOutput = 0;

    public AutoAlign(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(3));
        translationController.setTolerance(Units.inchesToMeters(4));

        setTagIDs(isReef);
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
        if (isAligned() && lastAlignCommand.equals(currentAlignCommand)) {
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

        lastAlignCommand = currentAlignCommand;
    }

    public void setTagIDs(boolean isReef) {
        if (RobotContainer.isRedAlliance()) {
            tagIDs = isReef ? FieldConstants.RED_REEF_TAG_IDS : FieldConstants.RED_CORAL_STATION_TAG_IDS;
        } else {
            tagIDs = isReef ? FieldConstants.BLUE_REEF_TAG_IDS : FieldConstants.BLUE_CORAL_STATION_TAG_IDS;
        }
        this.isReef = isReef;
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

    public void setCurrentAlignCommand(String curCommand) {
        currentAlignCommand = curCommand;
    }

    private static Map<Integer, Pose3d> loadTagPositions() {
        Map<Integer, Pose3d> tagMap = new HashMap<>();
        for (int tag : FieldConstants.RED_REEF_TAG_IDS) {
            tagMap.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        for (int tag : FieldConstants.BLUE_REEF_TAG_IDS) {
            tagMap.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        for (int tag : FieldConstants.RED_CORAL_STATION_TAG_IDS) {
            tagMap.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        for (int tag : FieldConstants.BLUE_CORAL_STATION_TAG_IDS) {
            tagMap.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        return tagMap;
    }

    private Command getAlignCommand(Drive drive, boolean isReef, double tx, String curCommand) {
        return new InstantCommand(() -> {
                    setCurrentAlignCommand(curCommand);
                    setTagIDs(isReef);
                    setBranchTx(tx);
                })
                .andThen(DriveCommands.joystickDrive(
                                drive, () -> getXVelocity(), () -> getYVelocity(), () -> getAngularVelocity(), true)
                        .alongWith(new RunCommand(() -> updateFieldRelativeAlignSpeeds())));
    }

    public Command reefAlignLeft(Drive drive) {
        return getAlignCommand(drive, true, AlignConstants.REEF_ALIGN_LEFT_TX, "Left Branch");
    }

    public Command reefAlignMid(Drive drive) {
        return getAlignCommand(drive, true, AlignConstants.REEF_ALIGN_MID_TX, "Mid Reef");
    }

    public Command reefAlignRight(Drive drive) {
        return getAlignCommand(drive, true, AlignConstants.REEF_ALIGN_RIGHT_TX, "Right Branch");
    }

    public Command stationAlign(Drive drive) {
        return getAlignCommand(drive, false, AlignConstants.STATION_ALIGN_TX, "Station");
    }

}