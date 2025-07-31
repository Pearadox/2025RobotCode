package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.util.simulation.AlgaeHandler;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

public class EndEffectorIOSim implements EndEffectorIO {
    private PearadoxTalonFX endEffector;
    private TalonFXSimState endEffectorSimState;

    // must be within 6 inches of the intake
    private static final double TRANSLATIONAL_TOLERANCE_METERS = Units.inchesToMeters(16);

    private boolean hasCoral = true;
    private boolean hasAlgae = false;

    // interpolates the game piece position from its original location to the robot
    private Timer intakingTimer = new Timer();
    private static final double INTAKING_TIME = 0.5;

    // cooldown between shooting and rerunning the intake
    private Timer shootingTimer = new Timer();
    private static final double SHOOTING_TIME = 1.25;

    // cooldown for dropping coral at the coral station
    private Timer droppingTimer = new Timer();
    private static final double DROP_COOLDOWN = 1.5;

    private Pose3d intookGamePiecePrevPose = Pose3d.kZero;

    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    private DoubleSupplier elevatorHeightSupplier;
    private DoubleSupplier armAngleSupplier;

    public EndEffectorIOSim(
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            DoubleSupplier elevatorHeightSupplier,
            DoubleSupplier armAngleSupplier) {

        this.poseSupplier = poseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
        this.elevatorHeightSupplier = elevatorHeightSupplier;
        this.armAngleSupplier = armAngleSupplier;

        endEffector = new PearadoxTalonFX(EndEffectorConstants.END_EFFECTOR_ID, NeutralModeValue.Brake, 60, false);

        BaseStatusSignal.setUpdateFrequencyForAll(
                ArmConstants.UPDATE_FREQ,
                endEffector.getPosition(),
                endEffector.getVelocity(),
                endEffector.getDutyCycle(),
                endEffector.getMotorVoltage(),
                endEffector.getTorqueCurrent(),
                endEffector.getSupplyCurrent(),
                endEffector.getStatorCurrent());

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.2;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        endEffector.getConfigurator().apply(slot0Configs);

        endEffectorSimState = endEffector.getSimState();
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        endEffectorSimState.setSupplyVoltage(12);

        inputs.positionRots = endEffector.getPosition().getValueAsDouble();
        inputs.velocityRps = endEffector.getVelocity().getValueAsDouble();

        inputs.appliedVolts = endEffector.getMotorVoltage().getValueAsDouble();

        inputs.statorCurrent = endEffector.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrent = endEffector.getSupplyCurrent().getValueAsDouble();

        inputs.hasCoral = this.hasCoral;

        if (inputs.appliedVolts <= -0.5) { // rollers spinning clockwise
            intakeCoralProjectiles();
            autoDropNearCS();
            shootAlgae();
        } else if (inputs.appliedVolts >= 0.5) { // rollers spinning counterclockwise
            shootCoral();
            intakeAlgae();
        }

        visualizeHeldGamePiece();
    }

    @Override
    public void setSpeed(double speed) {
        endEffector.set(speed);
    }

    private void intakeCoralProjectiles() {
        // cooldown between shooting and restarting the intake
        // so you don't immediately reintake what you just launched
        if (!checkAndResetTimer(shootingTimer, SHOOTING_TIME)) return;

        if (hasCoral) return; // max 1 coral

        Pose3d intakePose = getHeldCoralPose(); // the end effector is the intake

        var iterator = SimulatedArena.getInstance().gamePieceLaunched().iterator();
        while (iterator.hasNext()) {
            var gamePiece = iterator.next();
            if (gamePiece instanceof ReefscapeCoralOnFly) {
                if (checkTolerance(intakePose.minus(gamePiece.getPose3d()))) {
                    iterator.remove();
                    intookGamePiecePrevPose = gamePiece.getPose3d();
                    hasCoral = true;
                    intakingTimer.restart();
                    break;
                }
            }
        }
    }

    private static boolean checkTolerance(Transform3d difference) {
        return difference.getTranslation().getNorm() < TRANSLATIONAL_TOLERANCE_METERS;
    }

    private void visualizeHeldGamePiece() {
        Logger.recordOutput(
                "FieldSimulation/Held Coral", hasCoral ? interpolateHeldPose(getHeldCoralPose()) : Pose3d.kZero);
        Logger.recordOutput(
                "FieldSimulation/Held Algae", hasAlgae ? interpolateHeldPose(getHeldAlgaePose()) : Pose3d.kZero);
    }

    private Pose3d interpolateHeldPose(Pose3d targetPose) {
        if (intakingTimer.isRunning()) {
            if (intakingTimer.get() > INTAKING_TIME) {
                intakingTimer.stop();
                intakingTimer.reset();
            } else {
                return intookGamePiecePrevPose.interpolate(targetPose, intakingTimer.get() / INTAKING_TIME);
            }
        }
        return targetPose;
    }

    private Transform3d getHeldCoralTransform() {
        double armAngle = armAngleSupplier.getAsDouble();
        double elevatorHeight = elevatorHeightSupplier.getAsDouble();

        Translation3d pivotToEE = new Translation3d(SimulationConstants.PIVOT_TO_MIDDLE_OF_CORAL_RADIUS, 0, 0)
                .rotateBy(new Rotation3d(0, -armAngle - SimulationConstants.PIVOT_TO_MIDDLE_OF_CORAL_ANG_OFFSET, 0))
                .plus(new Translation3d(
                        0, SimulationConstants.CORAL_Y_OFFSET, elevatorHeight + SimulationConstants.ARM_CAD_ZERO_Z));

        Rotation3d rotation =
                new Rotation3d(0, -Math.PI - armAngle - SimulationConstants.PIVOT_ANGLE_TO_CORAL_ANGLE, 0);

        return new Transform3d(pivotToEE, rotation);
    }

    private Pose3d getHeldCoralPose() {
        Pose3d robotPose = new Pose3d(poseSupplier.get());
        return robotPose.transformBy(getHeldCoralTransform());
    }

    private void shootCoral() {
        if (!hasCoral) return;

        Transform3d eeTransform = getHeldCoralTransform();

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        // Obtain robot position from drive simulation
                        poseSupplier.get().getTranslation(),
                        // The scoring mechanism is installed at this position on the robot
                        eeTransform.getTranslation().toTranslation2d(),
                        // Obtain robot speed from drive simulation
                        chassisSpeedsSupplier.get(),
                        // Obtain robot facing from drive simulation
                        poseSupplier.get().getRotation(),
                        // The height at which the coral is ejected
                        eeTransform.getMeasureZ(),
                        // The initial speed of the coral
                        MetersPerSecond.of(-2),
                        // The coral is ejected at this angle
                        eeTransform.getRotation().getMeasureAngle().unaryMinus()));

        hasCoral = false;
        shootingTimer.restart();
    }

    // Drops a coral projectile from the coral station
    private void autoDropNearCS() {
        // don't drop if the robot has a coral
        if (hasCoral) return;

        // don't drop if the robot is moving
        ChassisSpeeds speeds = chassisSpeedsSupplier.get();
        if (Math.abs(speeds.vxMetersPerSecond + speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond) > 0.1) {
            return;
        }

        // don't drop if the robot is too far from the station
        Pose3d eePose = getHeldCoralPose();
        Pose2d nearestCS = eePose.toPose2d().nearest(FieldConstants.CORAL_STATIONS);
        Transform2d diff = eePose.toPose2d().minus(nearestCS);

        if (diff.getTranslation().getNorm() > TRANSLATIONAL_TOLERANCE_METERS) return;

        // don't drop if it's been too soon since the last drop
        if (!checkAndResetTimer(droppingTimer, DROP_COOLDOWN)) return;

        droppingTimer.restart();

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        // Coral Station Translation
                        nearestCS.getTranslation(),
                        // No additional offset
                        Translation2d.kZero,
                        // No additional velocity (the coral station isn't moving)
                        new ChassisSpeeds(),
                        // Coral station yaw
                        nearestCS.getRotation(),
                        // The height at which the coral is ejected
                        Meters.of(1),
                        // The initial speed of the coral
                        MetersPerSecond.of(1),
                        // The coral is ejected at this angle (the chute's pitch)
                        Degrees.of(-55)));
    }

    private Transform3d getHeldAlgaeTransform() {
        double armAngle = armAngleSupplier.getAsDouble();
        double elevatorHeight = elevatorHeightSupplier.getAsDouble();

        Translation3d pivotToEE = new Translation3d(SimulationConstants.PIVOT_TO_MIDDLE_OF_CORAL_RADIUS, 0, 0)
                .rotateBy(new Rotation3d(0, -armAngle, 0))
                .plus(new Translation3d(0, 0, elevatorHeight + SimulationConstants.ARM_CAD_ZERO_Z));

        Rotation3d rotation = new Rotation3d(0, -armAngle, 0);

        return new Transform3d(pivotToEE, rotation);
    }

    private Pose3d getHeldAlgaePose() {
        Pose3d robotPose = new Pose3d(poseSupplier.get());
        return robotPose.transformBy(getHeldAlgaeTransform());
    }

    private void intakeAlgae() {
        // cooldown between shooting and restarting the intake
        // so you don't immediately reintake what you just launched
        // if (shootingTimer.get() > SHOOTING_TIME) {
        //     shootingTimer.stop();
        //     shootingTimer.reset();
        //     disableIntake = false;
        // } else if (shootingTimer.get() > 0) {
        //     disableIntake = true;
        // }

        if (!checkAndResetTimer(shootingTimer, SHOOTING_TIME)) return;

        if (hasAlgae) return; // max 1 algae

        AlgaeHandler.getInstance().intake(getHeldAlgaePose()).ifPresent(algae -> {
            intookGamePiecePrevPose =
                    new Pose3d(algae.getTranslation(), getHeldAlgaePose().getRotation());
            hasAlgae = true;
            intakingTimer.restart();
        });
    }

    private void shootAlgae() {
        if (!hasAlgae) return;
        Transform3d eeTransform = getHeldAlgaeTransform();

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                        // Obtain robot position from drive simulation
                        poseSupplier.get().getTranslation(),
                        // The scoring mechanism is installed at this position on the robot
                        eeTransform.getTranslation().toTranslation2d(),
                        // Obtain robot speed from drive simulation
                        chassisSpeedsSupplier.get(),
                        // Obtain robot facing from drive simulation
                        poseSupplier.get().getRotation(),
                        // The height at which the coral is ejected
                        eeTransform.getMeasureZ(),
                        // The initial speed of the coral
                        MetersPerSecond.of(2.910),
                        // The coral is ejected at this angle
                        eeTransform.getRotation().getMeasureAngle().unaryMinus()));

        hasAlgae = false;
        shootingTimer.restart();
    }

    private static boolean checkAndResetTimer(Timer timer, double duration) {
        if (timer.get() > duration) {
            timer.stop();
            timer.reset();
            return true;
        } else if (timer.get() > 0) {
            return false;
        }
        return true; // Timer not started
    }
}
