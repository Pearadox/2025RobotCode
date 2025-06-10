package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
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

    private Timer intakingTimer = new Timer();
    private static final double INTAKING_TIME = 0.5;

    // cooldown between shooting and rerunning the intake
    private Timer shootingTimer = new Timer();
    private static final double SHOOTING_TIME = 1.25;
    private boolean disableIntake = false;

    private Timer droppingTimer = new Timer();
    private static final double DROP_COOLDOWN = 1.5;

    private Pose3d intookGamePiecePrevPose = Pose3d.kZero;
    private List<Pose3d> stagedAlgae = new ArrayList<>(List.of(FieldConstants.REEF_ALGAE_POSES));

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

        inputs.statorCurrent = endEffector.getStatorCurrent().getValueAsDouble() * (hasCoral || hasAlgae ? 1.0 : 0.1);
        inputs.supplyCurrent = endEffector.getSupplyCurrent().getValueAsDouble();

        if (inputs.appliedVolts <= -0.5) {
            intakeCoralProjectiles();
            autoDropNearCS();
            shootAlgae();
        } else if (inputs.appliedVolts >= 0.5) {
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
        if (shootingTimer.get() > SHOOTING_TIME) {
            shootingTimer.stop();
            shootingTimer.reset();
            disableIntake = false;
        } else if (shootingTimer.get() > 0) {
            disableIntake = true;
        }

        if (disableIntake || hasCoral) return; // max 1 coral

        Pose3d intakePose = getHeldCoralPose(); // the end effector is the intake
        Set<GamePieceProjectile> gamePieceProjectiles =
                SimulatedArena.getInstance().gamePieceLaunched();
        Set<GamePieceProjectile> toRemove = new HashSet<>();

        for (GamePieceProjectile gamePiece : gamePieceProjectiles) {
            if (gamePiece instanceof ReefscapeCoralOnFly) {
                if (checkTolerance(intakePose.minus(gamePiece.getPose3d()))) {
                    toRemove.add(gamePiece);
                    break; // max 1 coral
                }
            }
        }

        for (GamePieceProjectile gamePiece : toRemove) {
            gamePieceProjectiles.remove(gamePiece);
            intookGamePiecePrevPose = gamePiece.getPose3d();
            hasCoral = true;
            intakingTimer.restart();
            break; // max 1 coral
        }
    }

    private static boolean checkTolerance(Transform3d difference) {
        return difference.getTranslation().getNorm() < TRANSLATIONAL_TOLERANCE_METERS;
    }

    private void visualizeHeldGamePiece() {
        Logger.recordOutput("FieldSimulation/Pose", new Pose3d(poseSupplier.get()));
        Logger.recordOutput("FieldSimulation/Staged Algae", stagedAlgae.toArray(Pose3d[]::new));

        if (hasCoral) {
            if (intakingTimer.isRunning()) {
                if (intakingTimer.get() > INTAKING_TIME) {
                    intakingTimer.stop();
                    intakingTimer.reset();
                }
                Logger.recordOutput(
                        "FieldSimulation/Held Coral",
                        intookGamePiecePrevPose.interpolate(getHeldCoralPose(), intakingTimer.get() / INTAKING_TIME));
            } else {
                Logger.recordOutput("FieldSimulation/Held Coral", getHeldCoralPose());
            }
        } else {
            Logger.recordOutput("FieldSimulation/Held Coral", Pose3d.kZero);
        }

        if (hasAlgae) {
            if (intakingTimer.isRunning()) {
                if (intakingTimer.get() > INTAKING_TIME) {
                    intakingTimer.stop();
                    intakingTimer.reset();
                }
                Logger.recordOutput(
                        "FieldSimulation/Held Algae",
                        intookGamePiecePrevPose.interpolate(getHeldAlgaePose(), intakingTimer.get() / INTAKING_TIME));
            } else {
                Logger.recordOutput("FieldSimulation/Held Algae", getHeldAlgaePose());
            }
        } else {
            Logger.recordOutput("FieldSimulation/Held Algae", Pose3d.kZero);
        }
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
        // if (shootingTimer.get() > 0) return;

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
        disableIntake = true;
        shootingTimer.restart();
    }

    private void autoDropNearCS() {
        if (hasCoral) {
            return;
        }

        ChassisSpeeds speeds = chassisSpeedsSupplier.get();
        if (Math.abs(speeds.vxMetersPerSecond + speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond) > 0.1) {
            return;
        }

        Pose3d eePose = getHeldCoralPose();
        Pose2d nearestCS = eePose.toPose2d().nearest(FieldConstants.CORAL_STATIONS);
        Transform2d diff = eePose.toPose2d().minus(nearestCS);

        if (diff.getTranslation().getNorm() > TRANSLATIONAL_TOLERANCE_METERS) return;

        if (droppingTimer.get() > DROP_COOLDOWN) {
            droppingTimer.stop();
            droppingTimer.reset();
        } else if (droppingTimer.get() > 0) {
            return;
        }

        droppingTimer.restart();

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        // Obtain robot position from drive simulation
                        nearestCS.getTranslation(),
                        // The scoring mechanism is installed at this position on the robot
                        Translation2d.kZero,
                        // Obtain robot speed from drive simulation
                        new ChassisSpeeds(),
                        // Obtain robot facing from drive simulation
                        nearestCS.getRotation().plus(Rotation2d.k180deg),
                        // The height at which the coral is ejected
                        Meters.of(1),
                        // The initial speed of the coral
                        MetersPerSecond.of(1),
                        // The coral is ejected at this angle
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
        if (shootingTimer.get() > SHOOTING_TIME) {
            shootingTimer.stop();
            shootingTimer.reset();
            disableIntake = false;
        } else if (shootingTimer.get() > 0) {
            disableIntake = true;
        }

        if (disableIntake || hasAlgae) return; // max 1 algae

        Pose3d intakePose = getHeldAlgaePose(); // the end effector is the intake
        Set<Pose3d> toRemove = new HashSet<>();

        for (Pose3d algae : stagedAlgae) {
            if (checkTolerance(intakePose.minus(algae))) {
                toRemove.add(algae);
                break; // max 1 algae
            }
        }

        for (Pose3d gamePiece : toRemove) {
            stagedAlgae.remove(gamePiece);
            intookGamePiecePrevPose = gamePiece;
            hasAlgae = true;
            intakingTimer.restart();
            break; // max 1 algae
        }
    }

    private void shootAlgae() {
        if (!hasAlgae) return;
        // if (shootingTimer.get() > 0) return;

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
        disableIntake = true;
        shootingTimer.restart();
    }
}
