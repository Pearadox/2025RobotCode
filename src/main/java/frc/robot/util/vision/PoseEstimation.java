// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.SmarterDashboard;
import org.littletonrobotics.junction.Logger;

public class PoseEstimation {
    // private final SwerveDrivePoseEstimator poseEstimator;

    private final LimelightBackend[] backends;
    private final boolean[] backendToggles;

    private final TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(2);

    private static final double DIFFERENTIATION_TIME = Robot.defaultPeriodSecs;

    private GenericEntry robotPoseEntry = Shuffleboard.getTab("Swerve")
            .add("Robot Pose", new Pose2d().toString())
            .withSize(4, 1)
            .withPosition(4, 0)
            .getEntry();

    private static final NetworkTable llTable =
            NetworkTableInstance.getDefault().getTable(VisionConstants.LL_NAME);

    public PoseEstimation(/*SwerveDrivePoseEstimator poseEstimator*/ ) {
        // this.poseEstimator = poseEstimator;

        backends = new LimelightBackend[2];
        backendToggles = new boolean[2];

        backends[0] = new LimelightBackend(VisionConstants.LL_NAME, true);
        backends[1] = new LimelightBackend(VisionConstants.LL_B_NAME, true);
        backendToggles[0] = true;
        backendToggles[1] = true;
    }

    public void periodic(double angularSpeed) {
        boolean rejectUpdate = false;

        // setRobotOrientation(
        //         VisionConstants.LL_NAME,
        //         poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        //         0,
        //         0,
        //         0,
        //         0,
        //         0);

        if (Math.abs(angularSpeed) > 720) {
            rejectUpdate = true;
        }

        for (int i = 0; i < backends.length; i++) {
            if (backendToggles[i]) {
                // this is a hack to get around an issue in `SwerveDrivePoseEstimator`
                // where two measurements cannot share the same timestamp
                double timestampOffset = 1e-9 * i;

                if (backends[i].isValid() && !rejectUpdate) {
                    backends[i]
                            .getMeasurement()
                            .map((measurement) -> {
                                measurement.timestamp += timestampOffset;
                                return measurement;
                            })
                            .ifPresent(this::addVisionMeasurement);

                    backends[i]
                            .getMeasurement()
                            .map((measurement) -> {
                                measurement.timestamp += timestampOffset;
                                return measurement;
                            })
                            .ifPresent(this::loggingPose);
                }
            }
        }

        // QuestNav.periodic();
        // if (QuestNav.isConnected()) {
        //     addVisionMeasurement(QuestNav.getVisionMeasurement());
        // }

        double[] llStats = llTable.getEntry("hw").getDoubleArray(new double[4]);

        // note: docs are different from ip address dashboard
        // fps, temp, cpu usage, ram usage; not fps, cpu temp, ram usage, temp
        SmarterDashboard.putNumber("Limelight FPS", llStats[0], "Vision");
        SmarterDashboard.putNumber("Limelight Temp (C)", llStats[1], "Vision");
        SmarterDashboard.putNumber("Limelight CPU Usage", llStats[2], "Vision");
        SmarterDashboard.putNumber("Limelight Ram Usage", llStats[3], "Vision");

        // robotPoseEntry.setString(getEstimatedPose().toString());
        // poseHistory.addSample(Timer.getFPGATimestamp(), poseEstimator.getEstimatedPosition());
    }

    // public void updateOdometry(Rotation2d gyro, SwerveModulePosition[] modulePositions) {
    // poseEstimator.update(gyro, modulePositions);
    // }

    // public void updateWithTime(double timestamp, Rotation2d gyro, SwerveModulePosition[] modulePositions) {
    //     poseEstimator.updateWithTime(timestamp, gyro, modulePositions);
    // }

    // public Pose2d getEstimatedPose() {
    //     return poseEstimator.getEstimatedPosition();
    // }

    // public Translation2d getEstimatedVelocity() {
    //     double now = Timer.getFPGATimestamp();

    //     Translation2d current =
    //             poseHistory.getSample(now).orElseGet(Pose2d::new).getTranslation();
    //     Translation2d previous = poseHistory
    //             .getSample(now - DIFFERENTIATION_TIME)
    //             .orElseGet(Pose2d::new)
    //             .getTranslation();

    //     return current.minus(previous).div(DIFFERENTIATION_TIME);
    // }

    // public void resetPose(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
    //     poseEstimator.resetPosition(gyroAngle, modulePositions, pose);
    //     QuestNav.resetPose(pose);
    // }

    private void addVisionMeasurement(VisionBackend.Measurement measurement) {
        // poseEstimator.addVisionMeasurement(
        //         measurement.pose.toPose2d(), measurement.timestamp, measurement.stdDeviation);
        loggingPose(measurement);
        RobotContainer.drivetrain.addVisionMeasurement(
                measurement.pose.toPose2d(), measurement.timestamp, VisionConstants.MEGATAG2_LIMELIGHT_STD_DEV);
    }

    private void loggingPose(VisionBackend.Measurement measurement) {
        SmartDashboard.putString("Vision Pose", measurement.pose.toPose2d().toString());
        Logger.recordOutput("Drivetrain/Vision Pose", measurement.pose.toPose2d());
    }

    // private void setRobotOrientation(
    //         String limelightName,
    //         double yaw,
    //         double yawRate,
    //         double pitch,
    //         double pitchRate,
    //         double roll,
    //         double rollRate) {
    //     double[] entries = new double[6];
    //     entries[0] = yaw;
    //     entries[1] = yawRate;
    //     entries[2] = pitch;
    //     entries[3] = pitchRate;
    //     entries[4] = roll;
    //     entries[5] = rollRate;

    //     llTable.getEntry("robot_orientation_set").setDoubleArray(entries);
    // }
}
