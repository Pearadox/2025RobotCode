// package frc.robot.subsystems.Vision;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import frc.robot.util.QuestNav;

// public class VisionIOQuestNav implements VisionIO {
//     @Override
//     public void updateInputs(VisionIOInputs inputs) {
//         QuestNav.periodic();

//         if (inputs.connected = QuestNav.isConnected()) {
//             Pose2d pose = QuestNav.getRobotPose();

//             double displacement = Math.sqrt(Math.pow(pose.getX(), 2) + Math.pow(pose.getY(), 2));

//             inputs.poseObservations = new PoseObservation[] {
//                 new PoseObservation(
//                         QuestNav.getTimestamp(), new Pose3d(pose), 0.0, 1, displacement,
// PoseObservationType.QUESTNAV)
//             };
//         }
//     }
// }
