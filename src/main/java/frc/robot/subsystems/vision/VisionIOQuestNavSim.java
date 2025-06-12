package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.function.Supplier;

public class VisionIOQuestNavSim implements VisionIO {
    Supplier<Pose2d> poseSupplier;
    Pose2d resetPose;

    public VisionIOQuestNavSim(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = true;
        Pose2d pose = updatePose();
        if (pose == null) {
            inputs.poseObservations = new PoseObservation[0];
        } else {
            inputs.poseObservations = new PoseObservation[] {
                new PoseObservation(
                        System.currentTimeMillis() * 1e-6, new Pose3d(), 0.1, 1, 1, PoseObservationType.QUESTNAV)
            };
        }
    }

    // TODO
    private Pose2d updatePose() {
        return resetPose = poseSupplier.get();
    }
}
