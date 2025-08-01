package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class AutoAlign {
    private Pose2d targetPose = Pose2d.kZero;

    private Debouncer isAlignedDebouncer = new Debouncer(0.2);

    @Setter
    private Supplier<Pose2d> robotSupplier;

    public AutoAlign(Supplier<Pose2d> robotPoseSupplier) {
        this.robotSupplier = robotPoseSupplier;
    }

    public Command reefAlignLeft(Drive drive) {
        return reefAlign(drive, AlignConstants.LEFT_BRANCH_OFFSET);
    }

    public Command reefAlignMid(Drive drive) {
        return reefAlign(drive, AlignConstants.MID_OFFSET);
    }

    public Command reefAlignRight(Drive drive) {
        return reefAlign(drive, AlignConstants.RIGHT_BRANCH_OFFSET);
    }

    private Command reefAlign(Drive drive, Translation2d offset) {
        return new DriveToPose(drive, () -> findReefTargetPose(robotSupplier.get(), offset), robotSupplier);
    }

    private Pose2d findReefTargetPose(Pose2d currentPose, Translation2d offset) {
        Pose2d tagPose = currentPose.nearest(FieldConstants.REEF_TAGS);

        boolean isForwards = false;

        return targetPose =
                tagPose.transformBy(new Transform2d(offset, isForwards ? Rotation2d.k180deg : Rotation2d.kZero));
    }

    public Command stationAlign(Drive drive) {
        return new DriveToPose(drive, this::findStationTargetPose, robotSupplier);
    }

    private Pose2d findStationTargetPose() {
        return targetPose = robotSupplier
                .get()
                .nearest(FieldConstants.CORAL_STATIONS)
                .transformBy(new Transform2d(AlignConstants.STATION_OFFSET, Rotation2d.k180deg));
    }

    @AutoLogOutput
    public boolean isAligned() {
        Transform2d error = targetPose.minus(robotSupplier.get());

        return error.getTranslation().getNorm() < AlignConstants.ALIGN_TRANSLATION_TOLERANCE
                && error.getRotation().getRadians() < AlignConstants.ALIGN_ROT_TOLERANCE;
    }

    @AutoLogOutput
    public boolean isAlignedDebounced() {
        return isAlignedDebouncer.calculate(isAligned());
    }
}
