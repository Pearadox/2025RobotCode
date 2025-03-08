package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.util.vision.LimelightHelpers;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class AutoAlign {

    private Supplier<Pose2d> poseSupplier;

    private PIDController algaeReefSpeedController =
            new PIDController(AlignConstants.REEF_kP, AlignConstants.REEF_kI, AlignConstants.REEF_kD);

    private double alignSpeedStrafe = 0;
    private int currentReefAlignTagID = -1;
    private Map<Integer, Pose3d> tagPoses3d = new HashMap<Integer, Pose3d>();

    public AutoAlign(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
        tagPoses3d = getTagPoses();
    }

    public Map<Integer, Pose3d> getTagPoses() {
        Map<Integer, Pose3d> tagPoses = new HashMap<Integer, Pose3d>();
        for (int tag : FieldConstants.RED_REEF_TAG_IDS) {
            tagPoses.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        for (int tag : FieldConstants.BLUE_REEF_TAG_IDS) {
            tagPoses.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        return tagPoses;
    }

    private Rotation2d getTagAngle(int tagID) {

        if (tagPoses3d.containsKey(tagID)) {
            Pose3d tagPose = tagPoses3d.get(tagID);
            return new Rotation2d(tagPose.getRotation().getZ());
        } else return new Rotation2d(0);
    }

    public Rotation2d getAlignAngleReef() {
        int currentReefAlignTagID = getClosestAprilTag(
                RobotContainer.isRedAlliance() ? FieldConstants.RED_REEF_TAG_IDS : FieldConstants.BLUE_REEF_TAG_IDS,
                poseSupplier.get());

        return getTagAngle(currentReefAlignTagID);
    }

    private int getClosestAprilTag(int[] tagIDs, Pose2d robotPose) {
        double minDistance = Double.POSITIVE_INFINITY;
        int closestTagID = -1;

        // iterates through all tag IDs
        for (int i : tagIDs) {
            if (tagPoses3d.containsKey(i)) {
                Pose3d tagPose = tagPoses3d.get(i);

                // distance between robot pose and april tag
                double distance = tagPose.getTranslation()
                        .toTranslation2d()
                        .minus(robotPose.getTranslation())
                        .getNorm();

                if (distance < minDistance) {
                    closestTagID = i;
                    minDistance = distance;
                }
            }
        }

        return closestTagID;
    }

    public double getAlignStrafeSpeedPercent(double setPoint) {
        double tx = LimelightHelpers.getTX(VisionConstants.LL_NAME);
        double txError = tx - setPoint;

        // if the drivetrain isn't yet rotationally aligned, this affects the tx
        boolean withinRotRolerance = Math.abs(getAlignAngleReef()
                        .minus(poseSupplier.get().getRotation())
                        .getDegrees())
                < AlignConstants.ALIGN_ROT_TOLERANCE_DEGREES;
        Logger.recordOutput("Align/IsWithinRotTolerance", withinRotRolerance);

        boolean isValid = llIsValid() && withinRotRolerance;
        if (isValid) {
            // multiply error by kP to get the speed
            alignSpeedStrafe = -algaeReefSpeedController.calculate(tx, setPoint);
            alignSpeedStrafe += AlignConstants.ALIGN_KS * Math.signum(alignSpeedStrafe);
        } else {
            // reduce the current align speed by 1/4 each tick
            // this prevents it from suddenly stopping and starting when it loses sight of the tag
            alignSpeedStrafe *= AlignConstants.ALIGN_DAMPING_FACTOR;
        }

        Logger.recordOutput("Align/Strafe Speed", alignSpeedStrafe);
        Logger.recordOutput("Align/tx", tx);
        Logger.recordOutput("Align/tx Error", txError);

        return alignSpeedStrafe;
    }

    private boolean llIsValid() {
        boolean valid = LimelightHelpers.getTargetCount(VisionConstants.LL_NAME) == 1
                && LimelightHelpers.getFiducialID(VisionConstants.LL_NAME) == currentReefAlignTagID;
        Logger.recordOutput("Align/Valid", valid);
        return valid;
    }

    // public Map<Integer, Pose3d> getTagPoses3d() {
    //     return tagPoses3d;
    // }

    public ChassisSpeeds getFieldRelativeChassisSpeeds(double tx, double y, Rotation2d gyroAngle, double maxSpeed) {
        Logger.recordOutput("Align/Timestamp", System.currentTimeMillis());
        return ChassisSpeeds.fromRobotRelativeSpeeds(getAlignStrafeSpeedPercent(tx) * maxSpeed, y, 0, gyroAngle);
    }
}
