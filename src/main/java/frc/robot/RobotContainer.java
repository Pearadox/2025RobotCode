package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.RobotIdentity;
import frc.robot.util.SmarterDashboard;
import frc.robot.util.simulation.AlgaeHandler;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private Drive drive;
    private Vision vision;
    public static AutoAlign align;

    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    public static final XboxController driverController = new XboxController(0);
    public static final XboxController opController = new XboxController(1);

    // ---------------------------- Driver Controller --------------------------------- //

    private final JoystickButton resetHeading_Start =
            new JoystickButton(driverController, XboxController.Button.kStart.value);

    private final POVButton reefAlignLeft_PovLeft = new POVButton(driverController, 270);
    private final POVButton reefAlignCenter_PovDown = new POVButton(driverController, 180);
    private final POVButton reefAlignRight_PovRight = new POVButton(driverController, 90);
    private final POVButton stationAlign_PovUp = new POVButton(driverController, 0);

    private final JoystickButton slowMode_A = new JoystickButton(driverController, XboxController.Button.kA.value);

    private final Trigger strafe_Triggers = new Trigger(
            () -> Math.abs(driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis()) > 0.1);

    // ----------------------------- Op Controller -------------------------------- //

    public final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL: // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(Constants.TUNER_CONSTANTS.FrontLeft),
                        new ModuleIOTalonFX(Constants.TUNER_CONSTANTS.FrontRight),
                        new ModuleIOTalonFX(Constants.TUNER_CONSTANTS.BackLeft),
                        new ModuleIOTalonFX(Constants.TUNER_CONSTANTS.BackRight),
                        (robotPose) -> {});
                vision = new Vision(drive::accept, new VisionIOLimelight(camera0Name, drive::getRotation));
                // new VisionIOLimelight(camera1Name, drive::getRotation));
                break;

            case SIM: // Sim robot, instantiate physics sim IO implementations
                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(12, 2, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);
                vision = new Vision(drive::accept); // , new
                // VisionIOQuestNavSim(driveSimulation::getSimulatedDriveTrainPose));
                break;

            default: // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (robotPose) -> {});
                // elevator = new Elevator(new ElevatorIO() {});
                break;
        }

        align = new AutoAlign(drive::getPose);
        if (Constants.currentMode == Constants.Mode.SIM) {
            align.setRobotSupplier(driveSimulation::getSimulatedDriveTrainPose);
        }

        setDefaultCommands();
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("2B_IJ-CS-KL-CS-KL");
        autoChooser.addOption("Drive FF Ch", DriveCommands.feedforwardCharacterization(drive));
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();

        SmarterDashboard.putString("RoboRio Serial", RobotIdentity.getRoboRioSerial());
        SmarterDashboard.putString("Robot Identity", RobotIdentity.getRobotIdentityString());

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureBindings() {
        // ------------------------------- Driver Bindings ------------------------------- //

        // Reset gyro / odometry
        final Runnable resetOdometry = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
                : () -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
        resetHeading_Start.onTrue(Commands.runOnce(resetOdometry).ignoringDisable(true));

        reefAlignLeft_PovLeft.whileTrue(align.reefAlignLeft(drive));
        reefAlignRight_PovRight.whileTrue(align.reefAlignRight(drive));
        reefAlignCenter_PovDown.whileTrue(align.reefAlignMid(drive));
        stationAlign_PovUp.whileTrue(align.stationAlign(drive));

        strafe_Triggers.whileTrue(DriveCommands.joystickDrive(
                drive,
                () -> 0,
                () -> (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis()),
                () -> 0,
                false));

        slowMode_A.onTrue(new InstantCommand(() -> drive.changeSpeedMultiplier()));

        // ------------------------------- Operator Bindings ------------------------------- //
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    // -------------------------------- PathPlanner Commands -------------------------------- /w

    public void registerNamedCommands() {
        NamedCommands.registerCommand("Stop", new InstantCommand(() -> drive.stopWithX()));

        NamedCommands.registerCommand(
                "Auto Align Left",
                align.reefAlignLeft(drive).until(align::isAlignedDebounced).withTimeout(2));

        NamedCommands.registerCommand(
                "Auto Align Mid", align.reefAlignMid(drive).withTimeout(5));

        NamedCommands.registerCommand(
                "Auto Align Right",
                align.reefAlignRight(drive).until(align::isAlignedDebounced).withTimeout(2));

        NamedCommands.registerCommand(
                "Auto Align Station",
                align.stationAlign(drive).until(align::isAlignedDebounced).withTimeout(2));
    }

    // ----------------------------------- Default Commands -------------------------------- //

    public void setDefaultCommands() {
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY() * drive.getDriveMultiplier(),
                () -> -driverController.getLeftX() * drive.getDriveMultiplier(),
                () -> -driverController.getRightX() * drive.getTurnMultiplier(),
                true));
    }

    public void resetSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        drive.resetOdometry(new Pose2d(12, 2, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
        AlgaeHandler.getInstance().reset();
    }

    public void displaySimFieldToAdvantageScope() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/Pose", new Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput(
                "FieldSimulation/Staged Algae", AlgaeHandler.getInstance().periodic());
    }
}
