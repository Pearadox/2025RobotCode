package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotController;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class VisionIOQuestNav implements VisionIO {
    private QuestNav questNav;

    public VisionIOQuestNav() {
        questNav = new QuestNav();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        questNav.commandPeriodic();

        // First, Declare our geometrical transform from the robot center to the Quest
        Transform2d ROBOT_TO_QUEST = new Transform2d(); // TODO

        // Get the latest pose data frames from the Quest
        PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

        inputs.connected = poseFrames.length > 0;

        if (poseFrames.length > 0) {
            // Get the most recent Quest pose
            Pose2d questPose = poseFrames[poseFrames.length - 1].questPose();

            // Transform by the mount pose to get your robot pose
            Pose2d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

            double displacement = 1; // TODO

            inputs.poseObservations = new PoseObservation[] {
                new PoseObservation(
                        questNav.getAppTimestamp().orElse(RobotController.getFPGATime()),
                        new Pose3d(robotPose),
                        questNav.getLatency(),
                        1,
                        displacement,
                        PoseObservationType.QUESTNAV)
            };
        }
    }
}
