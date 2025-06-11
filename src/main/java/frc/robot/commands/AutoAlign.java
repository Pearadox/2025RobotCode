package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoAlign {
    private final Supplier<Pose2d> poseSupplier;

    private final PIDController translationController =
            new PIDController(AlignConstants.REEF_kP, AlignConstants.REEF_kI, AlignConstants.REEF_kD);
    private final PIDController rotationController =
            new PIDController(AlignConstants.ROT_REEF_kP, AlignConstants.ROT_REEF_kI, AlignConstants.ROT_REEF_kD);

    private double distanceError = 0;

    private Pose2d targetPose = new Pose2d();
    private Pose2d currentPose = new Pose2d();
    private final Map<Integer, Pose3d> tagPoses3d = loadTagPositions();

    private int[] tagIDs = {};
    private double currentBranchTx = AlignConstants.REEF_ALIGN_MID_TX;

    private double xVelocity = 0;
    private double yVelocity = 0;
    private double angularVelocity = 0;

    private Translation2d translationOutput = new Translation2d();
    private double rotationOutput = 0;

    public AutoAlign(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(3));
        translationController.setTolerance(1);
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
                .transformBy(new Transform2d(new Translation2d(currentBranchTx, 0), new Rotation2d()));

        updateControllerOutputs();
        updateVelocities();

        Logger.recordOutput("AutoAlign/TranslationError", distanceError);
        Logger.recordOutput("AutoAlign/RotationError", Math.toDegrees(rotationController.getError()));
        Logger.recordOutput("AutoAlign/TargetPose", targetPose);
        Logger.recordOutput("AutoAlign/CurrentBranchTag", currentTagID);
        Logger.recordOutput("xVelocity", xVelocity);
        Logger.recordOutput("yVelocity", yVelocity);
        Logger.recordOutput("angularVelocity", angularVelocity);
    }

    public void updateControllerOutputs() {
        Translation2d currentToTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
        distanceError = currentToTarget.getNorm();
        Rotation2d directionToTarget = currentToTarget.getAngle().plus(Rotation2d.k180deg);

        double translationMagnitude = translationController.calculate(0, distanceError);
        translationOutput = new Translation2d(translationMagnitude, directionToTarget);

        Logger.recordOutput("Align/Tmag", translationMagnitude);
        Logger.recordOutput("Align/dire", directionToTarget);

        rotationOutput = rotationController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    }

    public void updateVelocities() {
        xVelocity = translationOutput.getX();
        yVelocity = translationOutput.getY();
        angularVelocity = rotationOutput;
    }

    @AutoLogOutput
    public double get_xVelocity() {
        return xVelocity + 0.1 * Math.signum(xVelocity);
    }

    @AutoLogOutput
    public double get_yVelocity() {
        return yVelocity + 0.1 * Math.signum(yVelocity);
    }

    @AutoLogOutput
    public double get_angularVelocity() {
        return angularVelocity + 0.1 * Math.signum(angularVelocity);
    }

    public void setTagIDs(boolean isReef) {
        if (RobotContainer.isRedAlliance()) {
            tagIDs = isReef ? FieldConstants.RED_REEF_TAG_IDS : FieldConstants.RED_CORAL_STATION_TAG_IDS;
        } else {
            tagIDs = isReef ? FieldConstants.BLUE_REEF_TAG_IDS : FieldConstants.BLUE_CORAL_STATION_TAG_IDS;
        }
        System.out.println(tagIDs);
    }

    public void setBranchTx(double tx) {
        currentBranchTx = tx;
    }

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
}
