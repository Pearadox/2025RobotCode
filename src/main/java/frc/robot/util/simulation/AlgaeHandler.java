package frc.robot.util.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;

public class AlgaeHandler {
    private final List<Pose3d> stagedAlgae = new ArrayList<>();

    private static AlgaeHandler INSTANCE;

    public static AlgaeHandler getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new AlgaeHandler();
        }
        return INSTANCE;
    }

    private AlgaeHandler() {
        reset();
    }

    public void reset() {
        stagedAlgae.clear();
        stagedAlgae.addAll(List.of(FieldConstants.REEF_ALGAE_POSES));
    }

    public Pose3d[] periodic() {
        simulateNet();
        return getPose3ds();
    }

    public Optional<Pose3d> intake(Pose3d intakePose) {
        var iterator = stagedAlgae.iterator();
        while (iterator.hasNext()) {
            Pose3d algae = iterator.next();
            if (checkTolerance(intakePose.minus(algae))) {
                iterator.remove();
                return Optional.of(algae);
            }
        }
        return Optional.empty();
    }

    // collect launched algae projectiles
    public void simulateNet() {
        var iterator = SimulatedArena.getInstance().gamePieceLaunched().iterator();
        while (iterator.hasNext()) {
            var gamePiece = iterator.next();
            if (gamePiece instanceof ReefscapeAlgaeOnFly) {
                Pose3d algaePose = gamePiece.getPose3d();

                if (gamePiece.getVelocity3dMPS().getZ() < 0
                        && Math.abs(algaePose.getX() - FieldConstants.BARGE_X) < FieldConstants.BARGE_WIDTH
                        && Math.abs(algaePose.getZ() - FieldConstants.BARGE_HEIGHT)
                                < FieldConstants.BARGE_HEIGHT_TOLERANCE) {
                    stagedAlgae.add(new Pose3d(
                            FieldConstants.BARGE_X,
                            algaePose.getY() + gamePiece.getVelocity3dMPS().getY() / 2.0,
                            FieldConstants.BARGE_HEIGHT,
                            Rotation3d.kZero));
                    iterator.remove();
                }
            }
        }
    }

    public Pose3d[] getPose3ds() {
        return stagedAlgae.toArray(Pose3d[]::new);
    }

    private static boolean checkTolerance(Transform3d difference) {
        return difference.getTranslation().getNorm() < FieldConstants.EE_TOLERANCE;
    }
}
