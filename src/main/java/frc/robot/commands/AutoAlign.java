package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.util.vision.LimelightHelpers;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class AutoAlign {

    private Supplier<Pose2d> poseSupplier;

    private PIDController reefStrafeSpeedController =
            new PIDController(AlignConstants.REEF_kP, AlignConstants.REEF_kI, AlignConstants.REEF_kD);
    private PIDController reefForwardSpeedController =
            new PIDController(AlignConstants.REEF_Forward_kP, AlignConstants.REEF_kI, AlignConstants.REEF_kD);
    private PIDController reefRotationSpeedController =
            new PIDController(AlignConstants.ROT_REEF_kP, AlignConstants.ROT_REEF_kI, AlignConstants.ROT_REEF_kD);

    private double alignSpeedStrafe = 0;
    private double alignSpeedRotation = 0;
    private double alignSpeedForward = 0;
    private int currentReefAlignTagID = 18; // -1
    private int currentCSAlignTagID = 12; // -1
    private Map<Integer, Pose3d> tagPoses3d = getTagPoses();

    public AutoAlign(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
        reefRotationSpeedController.enableContinuousInput(-180, 180);
    }

    public Map<Integer, Pose3d> getTagPoses() {
        Map<Integer, Pose3d> tagPoses = new HashMap<Integer, Pose3d>();
        for (int tag : FieldConstants.RED_REEF_TAG_IDS) {
            tagPoses.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        for (int tag : FieldConstants.BLUE_REEF_TAG_IDS) {
            tagPoses.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        for (int tag : FieldConstants.RED_CORAL_STATION_TAG_IDS) {
            tagPoses.put(tag, VisionConstants.aprilTagLayout.getTagPose(tag).get());
        }
        for (int tag : FieldConstants.BLUE_CORAL_STATION_TAG_IDS) {
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

    private Pose2d getTagPose(int tagID) {

        if (tagPoses3d.containsKey(tagID)) {
            Pose3d tagPose = tagPoses3d.get(tagID);
            return tagPose.toPose2d();
        } else return new Pose2d();
    }

    public Rotation2d getAlignAngleReef() {
        if (!DriverStation.isAutonomous()) {
            setReefAlignTagIDtoClosest();
        }

        return getTagAngle(currentReefAlignTagID);
    }

    public Rotation2d getAlignAngleAlgaeReef() {
        currentReefAlignTagID = getClosestAprilTag(
                RobotContainer.isRedAlliance() ? FieldConstants.RED_REEF_TAG_IDS : FieldConstants.BLUE_REEF_TAG_IDS,
                poseSupplier.get());

        return new Rotation2d(getTagAngle(currentReefAlignTagID).getRadians() + Units.degreesToRadians(180));
    }

    public Rotation2d getAlignAngleStation() {
        currentCSAlignTagID = getClosestAprilTag(
                RobotContainer.isRedAlliance()
                        ? FieldConstants.RED_CORAL_STATION_TAG_IDS
                        : FieldConstants.BLUE_CORAL_STATION_TAG_IDS,
                poseSupplier.get());

        return new Rotation2d(getTagAngle(currentCSAlignTagID).getRadians() + +Units.degreesToRadians(180));
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

    public double getAlignStrafeSpeedPercent(double setPoint, int tagID) {
        // 3D transform of the robot in the coordinate system of the primary in-view AprilTag
        // (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees)
        // double[] targetRelativeRobotPose = LimelightHelpers.getBotPose_TargetSpace(VisionConstants.LL_NAME);
        // double tx = targetRelativeRobotPose[0];
        // double txError = tx - setPoint;

        Transform2d offset = poseSupplier.get().minus(getTagPose(tagID));

        // // if the drivetrain isn't yet rotationally aligned, this affects the tx
        // boolean withinRotRolerance = Math.abs(getAlignAngleReef()
        //                 .minus(poseSupplier.get().getRotation())
        //                 .getDegrees())
        //         < AlignConstants.ALIGN_ROT_TOLERANCE_DEGREES;
        // Logger.recordOutput("Align/IsWithinRotTolerance", withinRotRolerance);

        boolean isValid = llIsValid(); // && withinRotRolerance;
        if (true) {
            // multiply error by kP to get the speed
            alignSpeedStrafe = reefStrafeSpeedController.calculate(offset.getY(), setPoint);
            alignSpeedStrafe += AlignConstants.ALIGN_KS * Math.signum(alignSpeedStrafe);
        } else {
            // reduce the current align speed by 1/4 each tick
            // this prevents it from suddenly stopping and starting when it loses sight of the tag
            alignSpeedStrafe *= AlignConstants.ALIGN_DAMPING_FACTOR;
        }

        Logger.recordOutput("Align/Strafe Speed", alignSpeedStrafe);
        Logger.recordOutput("Align/Strafe Setpoint", setPoint);
        Logger.recordOutput("Align/Strafe Error", setPoint - offset.getY());
        // Logger.recordOutput("Align/tx", tx);
        // Logger.recordOutput("Align/tx Error", txError);

        return alignSpeedStrafe;
    }

    public double getAlignRotationSpeedPercent(Rotation2d targetAngle2d) {
        double robotAngle = poseSupplier.get().getRotation().getDegrees();
        double targetAngle = targetAngle2d.getDegrees();
        double rotationError = robotAngle - targetAngle;

        alignSpeedRotation = reefRotationSpeedController.calculate(robotAngle, targetAngle);

        Logger.recordOutput("Align/Rotation Speed", alignSpeedRotation);
        Logger.recordOutput("Align/Robot Angle", robotAngle);
        Logger.recordOutput("Align/Rotation Error", rotationError);

        return alignSpeedRotation;
    }

    public double getAlignForwardSpeedPercent(double setPoint, int tagID) {

        // double[] targetRelativeRobotPose = LimelightHelpers.getBotPose_TargetSpace(VisionConstants.LL_NAME);
        // double tz = targetRelativeRobotPose[2];
        // double tzError = tz - setPoint;

        Transform2d offset = poseSupplier.get().minus(getTagPose(tagID));

        if (true) {
            alignSpeedForward = reefForwardSpeedController.calculate(offset.getX(), setPoint);
        } else {
            // reduce the current align speed by 1/4 each tick
            // this prevents it from suddenly stopping and starting when it loses sight of the tag
            alignSpeedForward *= AlignConstants.ALIGN_DAMPING_FACTOR;
        }

        Logger.recordOutput("Align/Forward Speed", alignSpeedForward);
        // Logger.recordOutput("Align/ty", tz);
        // Logger.recordOutput("Align/ty Error", tzError);
        Logger.recordOutput("Align/x", offset.getX());
        Logger.recordOutput("Align/y", offset.getY());
        Logger.recordOutput("Align/Offset", offset);
        Logger.recordOutput("Align/Fwd Error", offset.getX() - setPoint);
        Logger.recordOutput("Align/Fwd Setpoint", setPoint);
        Logger.recordOutput("Align/Tag Pose", getTagPose(tagID));

        return alignSpeedForward;
    }

    public int getReefAlignTag() {
        return currentReefAlignTagID;
    }

    public int getStationAlignTag() {
        return currentCSAlignTagID;
    }

    private boolean llIsValid() {
        LimelightHelpers.setPriorityTagID(VisionConstants.LL_NAME, currentReefAlignTagID);
        boolean valid = LimelightHelpers.getTargetCount(VisionConstants.LL_NAME) >= 1
                && LimelightHelpers.getFiducialID(VisionConstants.LL_NAME) == currentReefAlignTagID;
        Logger.recordOutput("Align/Valid", valid);
        return valid;
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds(
            double tx, double ySpeed, Rotation2d gyroAngle, double maxSpeed, double maxAngularSpeed) {
        Logger.recordOutput("Align/Timestamp", System.currentTimeMillis());
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                getAlignStrafeSpeedPercent(tx, currentReefAlignTagID)
                        * maxSpeed, // getAlignStrafeSpeedPercent(tx) * maxSpeed
                ySpeed,
                getAlignRotationSpeedPercent(getAlignAngleReef()) * maxAngularSpeed,
                gyroAngle);
    }

    public Command getCSPathCommand(BooleanSupplier isProcessorSide) {
        try {
            if (isProcessorSide.getAsBoolean()) { // 2 & 12, processor side
                PathPlannerPath alignCSP = PathPlannerPath.fromPathFile("Align_CS_P");
                // Since AutoBuilder is configured, we can use it to build pathfinding commands
                return AutoBuilder.pathfindThenFollowPath(alignCSP, AlignConstants.PATH_CONSTRAINTS);
            } else {
                PathPlannerPath alignCSNP = PathPlannerPath.fromPathFile("Align_CS_NP");
                // Since AutoBuilder is configured, we can use it to build pathfinding commands
                return AutoBuilder.pathfindThenFollowPath(alignCSNP, AlignConstants.PATH_CONSTRAINTS);
            }
        } catch (Exception e) {
            e.printStackTrace();
            return Commands.print("align csp not found");
        }
    }

    public void setReefAlignTagIDtoClosest() {
        currentReefAlignTagID = getClosestAprilTag(
                RobotContainer.isRedAlliance() ? FieldConstants.RED_REEF_TAG_IDS : FieldConstants.BLUE_REEF_TAG_IDS,
                poseSupplier.get());
    }
}
