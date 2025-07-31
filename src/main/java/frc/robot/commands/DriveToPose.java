// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
    private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("Align/kP", AlignConstants.DRIVE_kP);
    private static final LoggedTunableNumber drivekI = new LoggedTunableNumber("Align/kI", AlignConstants.DRIVE_kI);
    private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("Align/kD", AlignConstants.DRIVE_kD);
    private static final LoggedTunableNumber driveMaxVel =
            new LoggedTunableNumber("Align/Max Vel", AlignConstants.MAX_DRIVE_VELOCITY);
    private static final LoggedTunableNumber driveMaxAcc =
            new LoggedTunableNumber("Align/Max Acc", AlignConstants.MAX_DRIVE_ACCELERATION);

    private static final LoggedTunableNumber rotkP = new LoggedTunableNumber("Align/Rot kP", AlignConstants.ROT_kP);
    private static final LoggedTunableNumber rotkI = new LoggedTunableNumber("Align/Rot kI", AlignConstants.ROT_kI);
    private static final LoggedTunableNumber rotkD = new LoggedTunableNumber("Align/Rot kD", AlignConstants.ROT_kD);
    private static final LoggedTunableNumber rotMaxVel =
            new LoggedTunableNumber("Align/Rot Max Vel", AlignConstants.MAX_ROT_VELOCITY);
    private static final LoggedTunableNumber rotMaxAcc =
            new LoggedTunableNumber("Align/Rot Max Acc", AlignConstants.MAX_ROT_ACCELERATION);

    private ProfiledPIDController translationController;
    private ProfiledPIDController rotationController;

    private Drive drive;
    private Supplier<Pose2d> targetSupplier;
    private Supplier<Pose2d> robotSupplier;

    /** Creates a new DriveToPose. */
    public DriveToPose(Drive drive, Supplier<Pose2d> target, Supplier<Pose2d> robot) {
        this.drive = drive;
        this.targetSupplier = target;
        this.robotSupplier = robot;

        translationController = new ProfiledPIDController(
                drivekP.get(),
                drivekI.get(),
                drivekD.get(),
                new TrapezoidProfile.Constraints(driveMaxVel.get(), driveMaxAcc.get()));

        rotationController = new ProfiledPIDController(
                rotkP.get(),
                rotkI.get(),
                rotkD.get(),
                new TrapezoidProfile.Constraints(rotMaxVel.get(), rotMaxAcc.get()));

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pose2d targetPose = targetSupplier.get();
        Pose2d currentPose = robotSupplier.get();
        ChassisSpeeds currentSpeeds = drive.getChassisSpeeds();

        Translation2d translationError = targetPose.minus(currentPose).getTranslation();
        Rotation2d directionToTarget = translationError.getAngle();
        double velocityTowardsTarget = currentSpeeds.vxMetersPerSecond * directionToTarget.getCos()
                + currentSpeeds.vyMetersPerSecond * directionToTarget.getSin();

        translationController.reset(translationError.getNorm(), -velocityTowardsTarget);
        rotationController.reset(robotSupplier.get().getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        updatePID();

        Pose2d targetPose = targetSupplier.get();
        Pose2d currentPose = robotSupplier.get();

        Translation2d translationError = targetPose.minus(currentPose).getTranslation();
        Rotation2d directionToTarget = translationError.getAngle();

        double translationOutput = translationController.calculate(translationError.getNorm(), 0);
        double rotationOutput = rotationController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        Translation2d translationVelocity = new Translation2d(-translationOutput, directionToTarget);

        drive.runVelocity(new ChassisSpeeds(translationVelocity.getX(), translationVelocity.getY(), rotationOutput));

        Logger.recordOutput("DriveToPose/Target", targetPose);
        Logger.recordOutput("DriveToPose/Translation Output", translationOutput);
        Logger.recordOutput("DriveToPose/Rotation Output", rotationOutput);
        Logger.recordOutput("DriveToPose/Translation Error", translationError.getNorm());
        Logger.recordOutput(
                "DriveToPose/Rotation Error",
                targetPose.getRotation().minus(currentPose.getRotation()).getDegrees());
        Logger.recordOutput("DriveToPose/Translation Velocity", translationVelocity);
        Logger.recordOutput("DriveToPose/Direction to Target", directionToTarget);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private void updatePID() {
        if (drivekP.hasChanged(hashCode()) || drivekI.hasChanged(hashCode()) || drivekD.hasChanged(hashCode())) {
            translationController.setPID(drivekP.get(), drivekI.get(), drivekD.get());
        }
        if (rotkP.hasChanged(hashCode()) || rotkI.hasChanged(hashCode()) || rotkD.hasChanged(hashCode())) {
            rotationController.setPID(rotkP.get(), rotkI.get(), rotkD.get());
        }
        if (driveMaxVel.hasChanged(hashCode()) || driveMaxAcc.hasChanged(hashCode())) {
            translationController.setConstraints(
                    new TrapezoidProfile.Constraints(driveMaxVel.get(), driveMaxAcc.get()));
        }
        if (rotMaxVel.hasChanged(hashCode()) || rotMaxAcc.hasChanged(hashCode())) {
            rotationController.setConstraints(new TrapezoidProfile.Constraints(rotMaxVel.get(), rotMaxAcc.get()));
        }
    }
}
