package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ArmHold;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.ElevatorHold;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorIOReal;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOQuestNavSim;
import frc.robot.util.simulation.MapleSimSwerveDrivetrain;
import frc.robot.util.vision.PoseEstimation;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private Elevator elevator;
    private Arm arm;
    private EndEffector endEffector;
    private Vision vision;
    private Climber climber;
    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final AutoAlign align = new AutoAlign(() -> drivetrain.getState().Pose);

    public static final PoseEstimation poseEstimation = new PoseEstimation();
    //     public static final LEDStrip ledstrip = LEDStrip.getInstance();

    private final SwerveRequest.FieldCentric fieldOrientedDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric robotOrientedDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controller
    public static final XboxController driverController = new XboxController(0);
    public static final XboxController opController = new XboxController(1);

    // Driver Controller
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

    // Op Controller
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
            case REAL: // Real robot, instantiate hardware IO implementations
                elevator = new Elevator(new ElevatorIOReal());
                arm = new Arm(new ArmIOReal());
                endEffector = new EndEffector(new EndEffectorIOReal());
                climber = new Climber(new ClimberIOReal());
                break;

            case SIM: // Sim robot, instantiate physics sim IO implementations
                elevator = new Elevator(new ElevatorIOSim());
                arm = new Arm(new ArmIOSim());
                endEffector = new EndEffector(new EndEffectorIOSim(
                        () -> MapleSimSwerveDrivetrain.getSimulatedPose(), // drivetrain.getState().Pose,
                        () -> drivetrain.getState().Speeds,
                        () -> elevator.getElevatorPositionMeters(),
                        () -> arm.getArmAngleRadsToHorizontal()));
                vision = new Vision(
                        drivetrain::addVisionMeasurement,
                        new VisionIOQuestNavSim(MapleSimSwerveDrivetrain::getSimulatedPose));
                climber = new Climber(new ClimberIOSim());
                break;

            default: // Replayed robot, disable IO implementations
                elevator = new Elevator(new ElevatorIO() {});
                arm = new Arm(new ArmIO() {});
                endEffector = new EndEffector(new EndEffectorIO() {});
                climber = new Climber(new ClimberIO() {});
                break;
        }

        setDefaultCommands();
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("2B_IJ-CS-KL-CS-KL");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();

        PathfindingCommand.warmupCommand().schedule();

        // Event Markers
        new EventTrigger("LevelStation")
                .onTrue(new InstantCommand(() -> elevator.setElevatorStationMode())
                        .andThen(new InstantCommand(() -> arm.setArmIntake())));
        new EventTrigger("LevelFour")
                .onTrue(new InstantCommand(() -> elevator.setElevatorLevelFourMode())
                        .andThen(new InstantCommand(() -> arm.setArmL4())));

        DriverStation.silenceJoystickConnectionWarning(Robot.isSimulation());
    }

    private void configureBindings() {
        // Driver Bindings
        resetHeading_Start.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // climbAlign_Y.whileTrue(drivetrain.applyRequest(
        //         () -> drive.withVelocityX(drivetrain.frontLimiter.calculate(-driverController.getLeftY())
        //                         * MaxSpeed
        //                         * drivetrain.getSpeedMultipler()) // Drive forward with negative Y (forward)
        //                 .withVelocityY(drivetrain.sideLimiter.calculate(-driverController.getLeftX())
        //                         * MaxSpeed
        //                         * drivetrain.getSpeedMultipler()) // Drive left with negative
        //                 // X (left)
        //                 .withRotationalRate(align.getAlignRotationSpeedPercent(
        //                                 (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
        //                                         ? Rotation2d.kCW_90deg
        //                                         : Rotation2d.kCCW_90deg)
        //                         * MaxAngularRate) // Drive
        //         // counterclockwise
        //         // with negative X
        //         // (left)
        //         ));

        stationAlign_PovUp.whileTrue(drivetrain.applyRequest(() -> robotOrientedDrive
                .withVelocityX(-align.getAlignForwardSpeedPercent(
                                AlignConstants.STATION_ALIGN_TZ, align.getStationAlignTag(), VisionConstants.LL_B_NAME)
                        * MaxSpeed)
                .withVelocityY(-align.getAlignStrafeSpeedPercent(
                                AlignConstants.STATION_ALIGN_TX, align.getStationAlignTag(), VisionConstants.LL_B_NAME)
                        * MaxSpeed)
                .withRotationalRate(
                        align.getAlignRotationSpeedPercent(align.getAlignAngleStation()) * MaxAngularRate)));
        // .alongWith(ledstrip.aligning(() -> align.isAligned())));

        reefAlignCenter_PovDown.whileTrue(drivetrain.applyRequest(() -> robotOrientedDrive
                .withVelocityX(align.getAlignForwardSpeedPercent(
                                AlignConstants.REEF_ALIGN_TZ, align.getReefAlignTag(), VisionConstants.LL_NAME)
                        * MaxSpeed)
                .withVelocityY(align.getAlignStrafeSpeedPercent(
                                AlignConstants.REEF_ALIGN_MID_TX, align.getReefAlignTag(), VisionConstants.LL_NAME)
                        * MaxSpeed)
                .withRotationalRate(align.getAlignRotationSpeedPercent(align.getAlignAngleReef()) * MaxAngularRate)));
        // .alongWith(ledstrip.aligning(() -> align.isAligned())));

        reefAlignLeft_PovLeft.whileTrue(drivetrain.applyRequest(() -> robotOrientedDrive
                .withVelocityX(align.getAlignForwardSpeedPercent(
                                AlignConstants.REEF_ALIGN_TZ, align.getReefAlignTag(), VisionConstants.LL_NAME)
                        * MaxSpeed)
                .withVelocityY(align.getAlignStrafeSpeedPercent(
                                AlignConstants.REEF_ALIGN_LEFT_TX, align.getReefAlignTag(), VisionConstants.LL_NAME)
                        * MaxSpeed)
                .withRotationalRate(align.getAlignRotationSpeedPercent(align.getAlignAngleReef()) * MaxAngularRate)));
        // .alongWith(ledstrip.aligning(() -> align.isAligned())));

        reefAlignRight_PovRight.whileTrue(
                drivetrain.applyRequest(() -> robotOrientedDrive
                        .withVelocityX(align.getAlignForwardSpeedPercent(
                                        AlignConstants.REEF_ALIGN_TZ, align.getReefAlignTag(), VisionConstants.LL_NAME)
                                * MaxSpeed)
                        .withVelocityY(align.getAlignStrafeSpeedPercent(
                                        AlignConstants.REEF_ALIGN_RIGHT_TX,
                                        align.getReefAlignTag(),
                                        VisionConstants.LL_NAME)
                                * MaxSpeed)
                        .withRotationalRate(
                                align.getAlignRotationSpeedPercent(align.getAlignAngleReef()) * MaxAngularRate))
                // .alongWith(ledstrip.aligning(() -> align.isAligned()))
                );

        slowMode_A.onTrue(new InstantCommand(() -> drivetrain.changeSpeedMultiplier()));
        zeroClimber_back.onTrue(new InstantCommand(() -> climber.zeroClimber()));

        strafe_Triggers.whileTrue(drivetrain.applyRequest(() -> robotOrientedDrive
                .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                .withVelocityY((driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis())
                        * 0.1
                        * MaxSpeed)
                .withRotationalRate(-driverController.getRightX() * MaxAngularRate)));

        // Operator Bindings
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
                .andThen(new InstantCommand(() -> endEffector.setCoral()))
                .andThen(new InstantCommand(() -> endEffector.stop())));
        algaeMode_RB.onTrue(new InstantCommand(() -> elevator.setAlgae())
                .andThen(new InstantCommand(() -> arm.setAlgae()))
                .andThen(new InstantCommand(() -> endEffector.setAlgae())));

        climberAdjustUp_PovUp
                .onTrue(new InstantCommand(() -> {
                    elevator.setElevatorStowedMode();
                    arm.setAlgae();
                    arm.setStowed();
                }))
                .whileTrue(new RunCommand(() -> climber.climberUp()))
                .onFalse(new InstantCommand(() -> climber.stop()));
        climberAdjustDown_PovDown
                .whileTrue(new RunCommand(() -> climber.climberDown()))
                .onFalse(new InstantCommand(() -> climber.stop()));
        climberStateInc_PovLeft.onTrue(new InstantCommand(() -> climber.decrementClimbState()));
        climberStateDec_PovRight.onTrue(new InstantCommand(() -> climber.incrementClimbState()));

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

        drivetrain.registerTelemetry(logger::telemeterize);
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

        NamedCommands.registerCommand("Outtake", new InstantCommand(() -> endEffector.coralOut()));
        NamedCommands.registerCommand(
                "Intake", new RunCommand(() -> endEffector.coralIn()).until(() -> endEffector.hasCoral()));
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

        NamedCommands.registerCommand(
                "Stop",
                drivetrain
                        .applyRequest(() -> robotOrientedDrive
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(0))
                        .withTimeout(0.05));

        NamedCommands.registerCommand(
                "Auto Align Left",
                new InstantCommand(() -> align.setReefAlignTagIDtoClosest())
                        .andThen((drivetrain.applyRequest(
                                        () -> robotOrientedDrive
                                                .withVelocityX(align.getAlignForwardSpeedPercent(
                                                                AlignConstants.REEF_ALIGN_TZ,
                                                                align.getReefAlignTag(),
                                                                VisionConstants.LL_NAME)
                                                        * MaxSpeed)
                                                .withVelocityY(align.getAlignStrafeSpeedPercent(
                                                                AlignConstants.REEF_ALIGN_LEFT_TX,
                                                                align.getReefAlignTag(),
                                                                VisionConstants.LL_NAME)
                                                        * MaxSpeed)
                                                .withRotationalRate(
                                                        align.getAlignRotationSpeedPercent(align.getAlignAngleReef())
                                                                * MaxAngularRate) // Drive counterclockwise with
                                        // negative X (left)
                                        )
                                // .alongWith(ledstrip.aligning(() -> align.isAligned()))
                                )
                                .until(() -> align.isAlignedTest())
                                .withTimeout(3))
                        .andThen(new WaitCommand(0.2)));

        NamedCommands.registerCommand(
                "Auto Align Mid",
                new InstantCommand(() -> align.setReefAlignTagIDtoClosest())
                        .andThen((drivetrain.applyRequest(() -> robotOrientedDrive
                                        .withVelocityX(align.getAlignForwardSpeedPercent(
                                                        AlignConstants.REEF_ALIGN_TZ,
                                                        align.getReefAlignTag(),
                                                        VisionConstants.LL_NAME)
                                                * MaxSpeed)
                                        .withVelocityY(align.getAlignStrafeSpeedPercent(
                                                        AlignConstants.REEF_ALIGN_MID_TX,
                                                        align.getReefAlignTag(),
                                                        VisionConstants.LL_NAME)
                                                * MaxSpeed)
                                        .withRotationalRate(
                                                align.getAlignRotationSpeedPercent(align.getAlignAngleReef())
                                                        * MaxAngularRate))
                                // .alongWith(ledstrip.aligning(() -> align.isAligned()))
                                )
                                .until(() -> align.isAlignedTest()))
                        .andThen(new WaitCommand(0.2)));

        NamedCommands.registerCommand(
                "Auto Align Right",
                new InstantCommand(() -> align.setReefAlignTagIDtoClosest())
                        .andThen((drivetrain.applyRequest(() -> robotOrientedDrive
                                        .withVelocityX(align.getAlignForwardSpeedPercent(
                                                        AlignConstants.REEF_ALIGN_TZ,
                                                        align.getReefAlignTag(),
                                                        VisionConstants.LL_NAME)
                                                * MaxSpeed)
                                        .withVelocityY(align.getAlignStrafeSpeedPercent(
                                                        AlignConstants.REEF_ALIGN_RIGHT_TX,
                                                        align.getReefAlignTag(),
                                                        VisionConstants.LL_NAME)
                                                * MaxSpeed)
                                        .withRotationalRate(
                                                align.getAlignRotationSpeedPercent(align.getAlignAngleReef())
                                                        * MaxAngularRate))
                                // .alongWith(ledstrip.aligning(() -> align.isAligned()))
                                )
                                .until(() -> align.isAlignedTest())
                                .withTimeout(3)
                                .andThen(new WaitCommand(0.2))));

        NamedCommands.registerCommand(
                "Auto Align Station",
                (drivetrain.applyRequest(() -> robotOrientedDrive
                                .withVelocityX(-align.getAlignForwardSpeedPercent(
                                                AlignConstants.STATION_ALIGN_TZ,
                                                align.getStationAlignTag(),
                                                VisionConstants.LL_B_NAME)
                                        * MaxSpeed)
                                .withVelocityY(-align.getAlignStrafeSpeedPercent(
                                                AlignConstants.STATION_ALIGN_TX,
                                                align.getStationAlignTag(),
                                                VisionConstants.LL_B_NAME)
                                        * MaxSpeed)
                                .withRotationalRate(align.getAlignRotationSpeedPercent(align.getAlignAngleStation())
                                        * MaxAngularRate)))
                        // .alongWith(ledstrip.aligning(() -> align.isAligned()))
                        .withTimeout(0.5));

        NamedCommands.registerCommand(
                "Set Algae",
                new InstantCommand(() -> elevator.setAlgae())
                        .andThen(new InstantCommand(() -> arm.setAlgae()))
                        .andThen(new InstantCommand(() -> endEffector.setAlgae())));

        NamedCommands.registerCommand(
                "Algae Intake", new RunCommand(() -> endEffector.algaeIn()).until(() -> endEffector.hasCoral()));
        NamedCommands.registerCommand("Algae Outtake", new InstantCommand(() -> endEffector.algaeOut()));
    }

    public void setDefaultCommands() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> fieldOrientedDrive
                .withVelocityX(drivetrain.frontLimiter.calculate(-driverController.getLeftY())
                        * MaxSpeed
                        * drivetrain.getSpeedMultipler())
                .withVelocityY(drivetrain.sideLimiter.calculate(-driverController.getLeftX())
                        * MaxSpeed
                        * drivetrain.getSpeedMultipler())
                .withRotationalRate(drivetrain.turnLimiter.calculate(-driverController.getRightX()) * MaxAngularRate)));

        elevator.setDefaultCommand(new ElevatorHold(elevator));
        arm.setDefaultCommand(new ArmHold(arm));
        // climber.setDefaultCommand(new ClimbCommand());
        // ledstrip.setDefaultCommand(ledstrip.defaultCommand(() -> endEffector.isCoral()));
    }

    public void resetSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        drivetrain.resetPose(new Pose2d(10.118, 1.18, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void displaySimFieldToAdvantageScope() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
