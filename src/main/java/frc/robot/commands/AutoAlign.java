package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.LoggedTunableNumber;
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

    private static LoggedTunableNumber tunablekP = new LoggedTunableNumber("Align/kP", AlignConstants.REEF_kP);
    private static LoggedTunableNumber tunablekI = new LoggedTunableNumber("Align/kI", AlignConstants.REEF_kI);
    private static LoggedTunableNumber tunablekD = new LoggedTunableNumber("Align/kD", AlignConstants.REEF_kD);

    private static LoggedTunableNumber tunableRotkP =
            new LoggedTunableNumber("Align/Rot kP", AlignConstants.ROT_REEF_kP);
    private static LoggedTunableNumber tunableRotkI =
            new LoggedTunableNumber("Align/Rot kI", AlignConstants.ROT_REEF_kI);
    private static LoggedTunableNumber tunableRotkD =
            new LoggedTunableNumber("Align/Rot kD", AlignConstants.ROT_REEF_kD);

    private final PIDController translationController =
            new PIDController(AlignConstants.REEF_kP, AlignConstants.REEF_kI, AlignConstants.REEF_kD);
    private final PIDController rotationController =
            new PIDController(AlignConstants.ROT_REEF_kP, AlignConstants.ROT_REEF_kI, AlignConstants.ROT_REEF_kD);

    private Debouncer isAlignedDebouncer = new Debouncer(0.2);

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
        rotationController.setTolerance(AlignConstants.ALIGN_ROT_TOLERANCE);
        translationController.setTolerance(AlignConstants.ALIGN_TRANSLATION_TOLERANCE);

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
                .transformBy(new Transform2d(
                        new Translation2d(currentBranchTz, currentBranchTx),
                        isReef ? Rotation2d.kZero : Rotation2d.k180deg));

        if (tunablekP.hasChanged(hashCode()) || tunablekI.hasChanged(hashCode()) || tunablekD.hasChanged(hashCode())) {
            translationController.setPID(tunablekP.get(), tunablekI.get(), tunablekD.get());
        }
        if (tunableRotkP.hasChanged(hashCode())
                || tunableRotkI.hasChanged(hashCode())
                || tunableRotkD.hasChanged(hashCode())) {
            rotationController.setPID(tunableRotkP.get(), tunableRotkI.get(), tunableRotkD.get());
        }

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
        if (RobotContainer.isRedAlliance()) {
            directionToTarget = directionToTarget.plus(Rotation2d.k180deg);
        }

        double translationMagnitude = MathUtil.clamp(translationController.calculate(0, distanceError), -0.8, 0.8);
        translationOutput = new Translation2d(translationMagnitude, directionToTarget);

        rotationOutput = rotationController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    }

    public void updateVelocities() {
        if (isAlignedDebounced()) {
            xVelocity = 0;
            yVelocity = 0;
            angularVelocity = 0;
        } else {
            xVelocity = translationOutput.getX();
            yVelocity = translationOutput.getY();
            angularVelocity = rotationOutput;

            xVelocity += AlignConstants.ALIGN_KS * Math.signum(xVelocity);
            yVelocity += AlignConstants.ALIGN_KS * Math.signum(yVelocity);
            angularVelocity += AlignConstants.ALIGN_KS * Math.signum(angularVelocity);

            xVelocity = square(xVelocity, RobotContainer.MaxSpeed * 0.5);
            yVelocity = square(yVelocity, RobotContainer.MaxSpeed * 0.5);
            angularVelocity = square(angularVelocity, RobotContainer.MaxAngularRate * 0.5);
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
        return translationController.atSetpoint()
                && rotationController.atSetpoint()
                && lastAlignCommand.equals(currentAlignCommand);
    }

    @AutoLogOutput(key = "AutoAlign/isAlignedDebounced")
    public boolean isAlignedDebounced() {
        return isAlignedDebouncer.calculate(isAligned());
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

    private Command getAlignCommand(
            CommandSwerveDrivetrain drive,
            SwerveRequest.FieldCentric req,
            boolean isReef,
            double tx,
            String curCommand) {
        return new InstantCommand(() -> {
                    setCurrentAlignCommand(curCommand + " " + Timer.getFPGATimestamp());
                    setTagIDs(isReef);
                    setBranchTx(tx);
                })
                .andThen(new RunCommand(() -> updateFieldRelativeAlignSpeeds())
                        .alongWith(drive.applyRequest(() -> req.withVelocityX(getXVelocity())
                                .withVelocityY(getYVelocity())
                                .withRotationalRate(getAngularVelocity()))));
    }

    public Command reefAlignLeft(CommandSwerveDrivetrain drive, SwerveRequest.FieldCentric req) {
        return getAlignCommand(drive, req, true, AlignConstants.REEF_ALIGN_LEFT_TX, "Left Branch");
    }

    public Command reefAlignMid(CommandSwerveDrivetrain drive, SwerveRequest.FieldCentric req) {
        return getAlignCommand(drive, req, true, AlignConstants.REEF_ALIGN_MID_TX, "Mid Reef");
    }

    public Command reefAlignRight(CommandSwerveDrivetrain drive, SwerveRequest.FieldCentric req) {
        return getAlignCommand(drive, req, true, AlignConstants.REEF_ALIGN_RIGHT_TX, "Right Branch");
    }

    public Command stationAlign(CommandSwerveDrivetrain drive, SwerveRequest.FieldCentric req) {
        return getAlignCommand(drive, req, false, AlignConstants.STATION_ALIGN_TX, "Station");
    }

    public static double square(double x, double max) {
        return (x * x * Math.signum(x)) * max;
    }
}
