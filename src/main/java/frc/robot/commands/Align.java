// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import frc.robot.Constants.AlignConstants;
// import frc.robot.Constants.FieldConstants;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.RobotContainer;
// import frc.robot.util.vision.LimelightHelpers;
// import java.util.function.Supplier;
// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// public class Align {
//     private Supplier<Pose2d> poseSupplier;

//     private int currentReefAlignTagID = -1;

//     private double alignSpeedForward = 0.0;
//     private double alignSpeedStrafe = 0.0;

//     private PIDController forwardPID = new PIDController(AlignConstants.ALIGN_FORWARD_KP, 0, 0);

//     private PIDController strafePID =
//             new PIDController(AlignConstants.ALIGN_STRAFE_KP, AlignConstants.ALIGN_STRAFE_KI, 0);

//     private LoggedNetworkNumber strafekP = new LoggedNetworkNumber("/SmartDashboard/strafe kp");
//     private LoggedNetworkNumber strafekI = new LoggedNetworkNumber("/SmartDashboard/strafe ki");
//     private LoggedNetworkNumber strafekD = new LoggedNetworkNumber("/SmartDashboard/strafe kd");

//     public Align(Supplier<Pose2d> poseSupplier) {
//         this.poseSupplier = poseSupplier;

//         strafekP.setDefault(strafePID.getP());
//         strafekI.setDefault(strafePID.getI());
//         strafekD.setDefault(strafePID.getD());
//     }

//     //   public Command getReefAlignCommand(Drive drive, double strafeSetpoint) {
//     //     // double strafeSetpoint;
//     //     // if (povLeft.getAsBoolean()) {
//     //     //   strafeSetpoint = AlignConstants.REEF_ALIGN_LEFT_TX;
//     //     // } else if (povRight.getAsBoolean()) {
//     //     //   strafeSetpoint = AlignConstants.REEF_ALIGN_RIGHT_TX;
//     //     // } else {
//     //     //   strafeSetpoint = AlignConstants.REEF_ALIGN_MID_TX;
//     //     // }

//     //     return DriveCommands.joystickDriveAtAngle(
//     //       drive,
//     //       () -> getAlignForwardSpeedPercent(AlignConstants.REEF_ALIGN_TY),
//     //       () -> getAlignStrafeSpeedPercent(strafeSetpoint),
//     //       () -> getAlignAngleReef(),
//     //       () -> false);
//     //   }

//     // returns the angle needed to align to the closest face of the reef
//     public Rotation2d getAlignAngleReef() {
//         currentReefAlignTagID = getClosestAprilTag(
//                 RobotContainer.isRedAlliance() ? FieldConstants.RED_REEF_TAG_IDS : FieldConstants.BLUE_REEF_TAG_IDS,
//                 poseSupplier.get());

//         return getTagAngle(currentReefAlignTagID);
//     }

//     // returns the angle needed to align to the given branch of the reef
//     public Rotation2d getAlignAngleReef(char branch) {
//         // if the branch is not a valid character A-L, just return the angle of the closest branch
//         if (!('A' <= branch && branch <= 'L')) return getAlignAngleReef();

//         int[] reefTagIDs = RobotContainer.isRedAlliance()
//                 ? FieldConstants.RED_CORAL_STATION_TAG_IDS
//                 : FieldConstants.BLUE_CORAL_STATION_TAG_IDS;

//         currentReefAlignTagID = reefTagIDs[(branch - 'A') / 2];
//         return getTagAngle(currentReefAlignTagID);
//     }

//     // returns the angle needed to align to the closest coral station
//     public Rotation2d getAlignAngleCoralStation() {
//         int closestCoralStationTagID = getClosestAprilTag(
//                 RobotContainer.isRedAlliance()
//                         ? FieldConstants.RED_CORAL_STATION_TAG_IDS
//                         : FieldConstants.BLUE_CORAL_STATION_TAG_IDS,
//                 poseSupplier.get());

//         // front of robot faces april tag
//         return getTagAngle(closestCoralStationTagID).minus(Rotation2d.k180deg);
//     }

//     // returns the closest april tag ID to the robot out of the provided array of april tag IDs
//     private static int getClosestAprilTag(int[] tagIDs, Pose2d robotPose) {
//         double minDistance = Double.POSITIVE_INFINITY;
//         int closestTagID = -1;

//         // iterates through all tag IDs
//         for (int i : tagIDs) {
//             // skips invalid tag IDs
//             var tagPoseOptional = VisionConstants.aprilTagLayout.getTagPose(i);
//             if (tagPoseOptional.isEmpty()) continue;

//             Pose3d tagPose = tagPoseOptional.get();

//             // distance between robot pose and april tag
//             double distance = tagPose.getTranslation()
//                     .toTranslation2d()
//                     .minus(robotPose.getTranslation())
//                     .getNorm(); // hypotenuse

//             if (distance < minDistance) {
//                 closestTagID = i;
//                 minDistance = distance;
//             }
//         }

//         return closestTagID;
//     }

//     // returns the yaw angle of the april tag
//     private static Rotation2d getTagAngle(int tagID) {
//         // if tagID is invalid, return an angle of 0
//         var tagPoseOptional = VisionConstants.aprilTagLayout.getTagPose(tagID);
//         if (tagPoseOptional.isEmpty()) return Rotation2d.kZero;

//         Pose3d tagPose = tagPoseOptional.get();
//         return new Rotation2d(tagPose.getRotation().getZ());
//     }

//     // returns a speed between -1 and 1
//     public double getAlignForwardSpeedPercent(double setPoint) {
//         double ty = LimelightHelpers.getTY(VisionConstants.LL_NAME);
//         double tyError = ty - setPoint;

//         if (llIsValid(tyError)) {
//             // multiply error by kP to get the speed
//             alignSpeedForward = forwardPID.calculate(ty, setPoint);
//         } else {
//             // reduce the current align speed by 1/4 each tick
//             // this prevents it from suddenly stopping and starting when it loses sight of the tag
//             alignSpeedForward *= AlignConstants.ALIGN_DAMPING_FACTOR;
//         }

//         // if (Math.abs(alignSpeedForward) < AlignConstants.ALIGN_SPEED_DEADBAND) alignSpeedForward = 0;

//         Logger.recordOutput("Align/Forward Speed", alignSpeedForward);
//         Logger.recordOutput("Align/ty", ty);
//         Logger.recordOutput("Align/ty Error", tyError);

//         return alignSpeedForward;
//     }

//     public double getAlignStrafeSpeedPercent(double setPoint) {
//         double tx = LimelightHelpers.getTX(VisionConstants.LL_NAME);
//         double txError = tx - setPoint;

//         // if the drivetrain isn't yet rotationally aligned, this affects the tx
//         boolean isValid = llIsValid(txError)
//                 && Math.abs(getAlignAngleReef()
//                                 .minus(poseSupplier.get().getRotation())
//                                 .getDegrees())
//                         < AlignConstants.ALIGN_ROT_TOLERANCE_DEGREES;

//         strafePID.setPID(strafekP.get(), strafekI.get(), strafekD.get());
//         // System.out.println(strafePID.getP() + " " + strafePID.getI() + " " + strafePID.getD());

//         if (isValid) {
//             // multiply error by kP to get the speed
//             alignSpeedStrafe = -strafePID.calculate(tx, setPoint);
//             alignSpeedStrafe += AlignConstants.ALIGN_KS * Math.signum(alignSpeedStrafe);
//         } else {
//             // reduce the current align speed by 1/4 each tick
//             // this prevents it from suddenly stopping and starting when it loses sight of the tag
//             alignSpeedStrafe *= AlignConstants.ALIGN_DAMPING_FACTOR;
//         }

//         // if (Math.abs(alignSpeedStrafe) < AlignConstants.ALIGN_SPEED_DEADBAND) alignSpeedStrafe = 0;

//         Logger.recordOutput("Align/Strafe Speed", alignSpeedStrafe);
//         Logger.recordOutput("Align/tx", tx);
//         Logger.recordOutput("Align/tx Error", txError);

//         return alignSpeedStrafe;
//     }

//     // if the error is too small, you do not see an april tag, or the current tagID is wrong
//     private boolean llIsValid(double error) {
//         return // Math.abs(error) > AlignConstants.ALIGN_TOLERANCE_PIXELS
//         LimelightHelpers.getTargetCount(VisionConstants.LL_NAME) == 1
//                 && LimelightHelpers.getFiducialID(VisionConstants.LL_NAME) == currentReefAlignTagID;
//     }

//     public ChassisSpeeds getFieldRelativeChassisSpeeds(double tx, double ty, Rotation2d gyroAngle, double maxSpeed) {
//         return ChassisSpeeds.fromRobotRelativeSpeeds(
//                 getAlignStrafeSpeedPercent(tx) * maxSpeed, getAlignForwardSpeedPercent(ty) * maxSpeed, 0, gyroAngle);
//     }
// }
