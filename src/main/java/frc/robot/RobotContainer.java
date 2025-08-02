package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmHold;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorHold;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.endeffector.EndEffectorIOReal;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
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
    private Elevator elevator;
    public static Arm arm;
    private EndEffector endEffector;
    private Vision vision;
    private Climber climber;
    public static AutoAlign align;

    private SwerveDriveSimulation driveSimulation = null;

    //     public static final LEDStrip ledstrip = LEDStrip.getInstance();

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
    private final JoystickButton zeroClimber_back =
            new JoystickButton(driverController, XboxController.Button.kBack.value);

    private final Trigger strafe_Triggers = new Trigger(
            () -> Math.abs(driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis()) > 0.1);

    // ----------------------------- Op Controller -------------------------------- //

    private final JoystickButton homeElevator_Start =
            new JoystickButton(opController, XboxController.Button.kStart.value);
    private final JoystickButton station_Back = new JoystickButton(opController, XboxController.Button.kBack.value);
    private final JoystickButton levelFour_Y = new JoystickButton(opController, XboxController.Button.kY.value);
    private final JoystickButton levelThree_B = new JoystickButton(opController, XboxController.Button.kB.value);
    private final JoystickButton levelTwo_X = new JoystickButton(opController, XboxController.Button.kX.value);
    private final JoystickButton stow_A = new JoystickButton(opController, XboxController.Button.kA.value);

    private final JoystickButton coralMode_LB =
            new JoystickButton(opController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton algaeMode_RB =
            new JoystickButton(opController, XboxController.Button.kRightBumper.value);

    private final POVButton climberStateInc_PovLeft = new POVButton(opController, 270);
    private final POVButton climberStateDec_PovRight = new POVButton(opController, 90);
    private final POVButton climberAdjustUp_PovUp = new POVButton(opController, 0);
    private final POVButton climberAdjustDown_PovDown = new POVButton(opController, 180);

    private final Trigger elevatorAdjust = new Trigger(() -> Math.abs(opController.getLeftY()) > 0.9);
    private final Trigger armAdjust = new Trigger(() -> Math.abs(opController.getRightX()) > 0.9);
    private final Trigger alignAdjust = new Trigger(() -> Math.abs(opController.getLeftTriggerAxis()) > 0.9);

    public final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        switch (Constants.currentMode) {

                // Real robot, instantiate hardware IO implementations

            case REAL:
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(Constants.TUNER_CONSTANTS.FrontLeft),
                        new ModuleIOTalonFX(Constants.TUNER_CONSTANTS.FrontRight),
                        new ModuleIOTalonFX(Constants.TUNER_CONSTANTS.BackLeft),
                        new ModuleIOTalonFX(Constants.TUNER_CONSTANTS.BackRight),
                        (robotPose) -> {});
                vision = new Vision(drive::accept, new VisionIOLimelight(camera0Name, drive::getRotation));
                // new VisionIOLimelight(camera1Name, drive::getRotation));
                elevator = new Elevator(new ElevatorIOReal());
                arm = new Arm(new ArmIOReal());
                endEffector = new EndEffector(new EndEffectorIOReal());
                climber = new Climber(new ClimberIOReal());
                break;

                // Sim robot, instantiate physics sim IO implementations

            case SIM:
                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(12, 2, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);

                elevator = new Elevator(new ElevatorIOSim());
                arm = new Arm(new ArmIOSim());
                endEffector = new EndEffector(new EndEffectorIOSim(
                        () -> driveSimulation.getSimulatedDriveTrainPose(), // drivetrain.getState().Pose,
                        () -> driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        () -> elevator.getElevatorPositionMeters(),
                        () -> arm.getArmAngleRadsToHorizontal()));
                vision = new Vision(drive::accept); // , new
                // VisionIOQuestNavSim(driveSimulation::getSimulatedDriveTrainPose));
                climber = new Climber(new ClimberIOSim());
                break;

                // Replayed robot, disable IO implementations

            default:
                elevator = new Elevator(new ElevatorIO() {});
                arm = new Arm(new ArmIO() {});
                endEffector = new EndEffector(new EndEffectorIO() {});
                climber = new Climber(new ClimberIO() {});
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

        // ------------------------------ Event Markers ---------------------------------- //

        new EventTrigger("LevelStation")
                .onTrue(new InstantCommand(() -> elevator.setElevatorStationMode())
                        .andThen(new InstantCommand(() -> arm.setArmIntake())));
        new EventTrigger("LevelFour")
                .onTrue(new InstantCommand(() -> elevator.setElevatorLevelFourMode())
                        .andThen(new InstantCommand(() -> arm.setArmL4())));

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
        zeroClimber_back.onTrue(new InstantCommand(() -> climber.zeroClimber()));

        // ------------------------------- Operator Bindings ------------------------------- //

        homeElevator_Start
                .whileTrue(new InstantCommand(() -> elevator.setZeroing(true)))
                .onFalse(new InstantCommand(() -> elevator.stopElevator())
                        .withTimeout(0.2)
                        .andThen(new InstantCommand(() -> elevator.zeroElevator())
                                .andThen(new InstantCommand(() -> elevator.setZeroing(false)))));

        station_Back.onTrue(new InstantCommand(() -> elevator.setElevatorStationMode())
                .andThen(new ConditionalCommand(
                        new WaitCommand(0.2), Commands.none(), () -> arm.getArmMode() == ArmMode.Stowed))
                .andThen(new InstantCommand(() -> arm.setArmIntake())));
        stow_A.onTrue(new InstantCommand(() -> arm.setStowed())
                .andThen(new WaitCommand(1))
                .andThen(new InstantCommand(() -> elevator.setElevatorStowedMode())));
        levelTwo_X.onTrue(new InstantCommand(() -> elevator.setElevatorLevelTwoMode())
                .andThen(new ConditionalCommand(
                        new WaitCommand(0.2), Commands.none(), () -> arm.getArmMode() == ArmMode.Stowed))
                .andThen(new InstantCommand(() -> arm.setArmL2())));
        levelThree_B.onTrue(new InstantCommand(() -> elevator.setElevatorLevelThreeMode())
                .andThen(new ConditionalCommand(
                        new WaitCommand(0.2), Commands.none(), () -> arm.getArmMode() == ArmMode.Stowed))
                .andThen(new InstantCommand(() -> arm.setArmL3())));
        levelFour_Y.onTrue(new InstantCommand(() -> elevator.setElevatorLevelFourMode())
                .andThen(new ConditionalCommand(
                        new WaitCommand(0.2), Commands.none(), () -> arm.getArmMode() == ArmMode.Stowed))
                .andThen(new InstantCommand(() -> arm.setArmL4())));

        coralMode_LB.onTrue(new InstantCommand(() -> elevator.setCoral())
                .andThen(new InstantCommand(() -> arm.setCoral()))
                .andThen(new InstantCommand(() -> endEffector.setCoralMode()))
                .andThen(new InstantCommand(() -> endEffector.stopEE())));
        algaeMode_RB.onTrue(new InstantCommand(() -> elevator.setAlgae())
                .andThen(new InstantCommand(() -> arm.setAlgae()))
                .andThen(new InstantCommand(() -> endEffector.setAlgaeMode())));

        climberAdjustUp_PovUp
                .whileTrue(new RunCommand(() -> climber.climberUp()))
                .onFalse(new InstantCommand(() -> climber.stop()));
        climberAdjustDown_PovDown
                .whileTrue(new RunCommand(() -> climber.climberDown()))
                .onFalse(new InstantCommand(() -> climber.stop()));
        climberStateInc_PovLeft.onTrue(new InstantCommand(() -> climber.decrementClimbState()));
        climberStateDec_PovRight.onTrue(new InstantCommand(() -> {
                    elevator.setElevatorStowedMode();
                    arm.setAlgae();
                    arm.setStowed();
                })
                .andThen(new WaitCommand(0.2))
                .andThen(new InstantCommand(() -> climber.incrementClimbState())));

        elevatorAdjust.whileTrue(
                new RunCommand(() -> elevator.changeElevatorOffset(.01 * Math.signum(-opController.getLeftY()))));
        armAdjust.whileTrue(new RunCommand(() -> arm.armAdjust(.01 * Math.signum(opController.getRightX()))));
        alignAdjust
                .onTrue(new InstantCommand(() -> arm.setAligning(true))
                        .andThen(new InstantCommand(() -> elevator.setAligning(true))))
                .onFalse(new InstantCommand(() -> arm.setAligning(false))
                        .andThen(new InstantCommand(() -> elevator.setAligning(false))));

        // if (Constants.currentMode == Constants.Mode.SIM) {
        //     csDropLB.onTrue(new InstantCommand(() -> dropCoralFromStation(false)).ignoringDisable(true));
        //     csDropRB.onTrue(new InstantCommand(() -> dropCoralFromStation(true)).ignoringDisable(true));
        // }
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
        NamedCommands.registerCommand(
                "LevelStation",
                new InstantCommand(() -> elevator.setElevatorStationMode())
                        .andThen(new InstantCommand(() -> arm.setArmIntake())));
        NamedCommands.registerCommand("ElevatorStowedMode", new InstantCommand(() -> elevator.setElevatorStowedMode()));
        NamedCommands.registerCommand(
                "LevelTwo",
                new InstantCommand(() -> elevator.setElevatorLevelTwoMode())
                        .andThen(new InstantCommand(() -> arm.setArmL2())));
        NamedCommands.registerCommand(
                "LevelThree",
                new InstantCommand(() -> elevator.setElevatorLevelThreeMode())
                        .andThen(new InstantCommand(() -> arm.setArmL3())));
        NamedCommands.registerCommand(
                "LevelFour",
                new InstantCommand(() -> elevator.setElevatorLevelFourMode())
                        .andThen(new ConditionalCommand(
                                new WaitCommand(0.2), Commands.none(), () -> arm.getArmMode() == ArmMode.Stowed))
                        .andThen(new InstantCommand(() -> arm.setArmL4())));

        NamedCommands.registerCommand("Outtake", new InstantCommand(() -> endEffector.outtakeCoral()));
        NamedCommands.registerCommand(
                "Intake", new RunCommand(() -> endEffector.intakeCoral()).until(() -> endEffector.hasCoral()));
        NamedCommands.registerCommand("Stop EE", new InstantCommand(() -> endEffector.stopCoral()));
        NamedCommands.registerCommand("Hold Coral", new InstantCommand(() -> endEffector.holdCoral()));
        NamedCommands.registerCommand(
                "Home Elevator",
                new InstantCommand(() -> elevator.setZeroing(true))
                        .withTimeout(2.5)
                        .andThen(new InstantCommand(() -> elevator.stopElevator()))
                        .withTimeout(0.2)
                        .andThen(new InstantCommand(() -> elevator.zeroElevator()))
                        .andThen(new InstantCommand(() -> elevator.setZeroing(false))));

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

        NamedCommands.registerCommand(
                "Set Algae",
                new InstantCommand(() -> elevator.setAlgae())
                        .andThen(new InstantCommand(() -> arm.setAlgae()))
                        .andThen(new InstantCommand(() -> endEffector.setAlgaeMode())));
        NamedCommands.registerCommand(
                "Set Coral",
                new InstantCommand(() -> elevator.setAlgae())
                        .andThen(new InstantCommand(() -> arm.setCoral()))
                        .andThen(new InstantCommand(() -> endEffector.setCoralMode())));

        NamedCommands.registerCommand(
                "Algae Intake", new RunCommand(() -> endEffector.intakeAlgae()).until(() -> endEffector.hasCoral()));
        NamedCommands.registerCommand("Algae Outtake", new InstantCommand(() -> endEffector.outtakeAlgae()));
    }

    // ----------------------------------- Default Commands -------------------------------- //

    public void setDefaultCommands() {
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY() * drive.getDriveMultiplier(),
                () -> -driverController.getLeftX() * drive.getDriveMultiplier(),
                () -> -driverController.getRightX() * drive.getTurnMultiplier(),
                true));

        elevator.setDefaultCommand(new ElevatorHold(elevator));
        arm.setDefaultCommand(new ArmHold(arm));
        // climber.setDefaultCommand(new ClimbCommand());
        // ledstrip.setDefaultCommand(ledstrip.defaultCommand(() -> endEffector.isCoral()));
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
