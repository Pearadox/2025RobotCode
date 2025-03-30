// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmHold;
// import frc.robot.commands.AutoAlign;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ElevatorHold;
import frc.robot.commands.GroundIntakeHold;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.LEDStrip;
import frc.robot.util.vision.PoseEstimation;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final Elevator elevator = Elevator.getInstance();
    public static final Arm arm = Arm.getInstance();
    public static final EndEffector endEffector = EndEffector.getInstance();
    public static final GroundIntake gIntake = GroundIntake.getInstance();
    public static final Climber climber = Climber.getInstance();
    public static final LEDStrip ledstrip = LEDStrip.getInstance();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric robotOrientedDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle pointTowards = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final SendableChooser<Command> autoChooser;

    //     private final AutoAlign align;

    public static final PoseEstimation poseEstimation = new PoseEstimation();

    // Driver Controller
    public static final XboxController driverController = new XboxController(0);
    private final Trigger strafeTriggers = new Trigger(
            () -> Math.abs(driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis()) > 0.1);
    private final JoystickButton resetHeading_Start =
            new JoystickButton(driverController, XboxController.Button.kStart.value);
    private final JoystickButton handoff_back = new JoystickButton(driverController, XboxController.Button.kBack.value);
    private final POVButton alignPovLeft = new POVButton(driverController, 270);
    private final POVButton alignPovDown = new POVButton(driverController, 180);
    private final POVButton alignPovRight = new POVButton(driverController, 90);
    private final POVButton alignPOVUp = new POVButton(driverController, 0);
    private final JoystickButton intake_LB =
            new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton outtake_RB =
            new JoystickButton(driverController, XboxController.Button.kRightBumper.value);

    // Op Controller
    public static final XboxController opController = new XboxController(1);
    private final JoystickButton zeroElevator_Start =
            new JoystickButton(opController, XboxController.Button.kStart.value);
    private final JoystickButton coralMode_LB =
            new JoystickButton(opController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton algaeMode_RB =
            new JoystickButton(opController, XboxController.Button.kRightBumper.value);
    private final JoystickButton levelFour_Y = new JoystickButton(opController, XboxController.Button.kY.value);
    private final JoystickButton levelThree_B = new JoystickButton(opController, XboxController.Button.kB.value);
    private final JoystickButton levelTwo_X = new JoystickButton(opController, XboxController.Button.kX.value);
    private final JoystickButton station_Back = new JoystickButton(opController, XboxController.Button.kBack.value);
    private final JoystickButton stow_A = new JoystickButton(opController, XboxController.Button.kA.value);
    private final Trigger gIntakeDeploy_LT = new Trigger(() -> opController.getLeftTriggerAxis() > 0.6);
    private final Trigger gIntakeShoot_RT = new Trigger(() -> opController.getRightTriggerAxis() > 0.6);
    private final POVButton zeroClimber_PovLeft = new POVButton(opController, 270);
    private final POVButton deployClimber_PovUp = new POVButton(opController, 0);
    private final POVButton retractClimber_PovDown = new POVButton(opController, 180);
    private final Trigger elevatorAdjust = new Trigger(() -> Math.abs(opController.getLeftY()) > 0.9);
    private final Trigger armAdjust = new Trigger(() -> Math.abs(opController.getRightX()) > 0.9);

    public RobotContainer() {
        setDefaultCommands();
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("2R_EF-L4");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();

        // align = new AutoAlign(() -> drivetrain.getState().Pose);
        PathfindingCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        zeroElevator_Start
                .whileTrue(new InstantCommand(() -> elevator.setZeroing(true)))
                .onFalse(new InstantCommand(() -> elevator.zeroElevator())
                        .andThen(new InstantCommand(() -> elevator.setZeroing(false))));

        // alignPOVUp.whileTrue(drivetrain.applyRequest(
        //         () -> robotOrientedDrive
        //                 .withVelocityX(-align.getAlignForwardSpeedPercent(
        //                                 AlignConstants.STATION_ALIGN_TZ, align.getStationAlignTag())
        //                         * MaxSpeed) // Drive forward with negative Y (forward)
        //                 .withVelocityY(-align.getAlignStrafeSpeedPercent(
        //                                 AlignConstants.STATION_ALIGN_TX, align.getStationAlignTag())
        //                         * MaxSpeed) // Drive left with negative X (left)
        //                 .withRotationalRate(align.getAlignRotationSpeedPercent(align.getAlignAngleStation())
        //                         * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //         ));

        // alignPovDown.whileTrue(drivetrain.applyRequest(
        //         () -> robotOrientedDrive
        //                 .withVelocityX(
        //                         align.getAlignForwardSpeedPercent(AlignConstants.REEF_ALIGN_TZ,
        // align.getReefAlignTag())
        //                                 * MaxSpeed) // Drive forward with negative Y (forward)
        //                 .withVelocityY(align.getAlignStrafeSpeedPercent(
        //                                 AlignConstants.REEF_ALIGN_MID_TX, align.getReefAlignTag())
        //                         * MaxSpeed) // Drive left with negative X (left)
        //                 .withRotationalRate(align.getAlignRotationSpeedPercent(align.getAlignAngleReef())
        //                         * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //         ));

        // alignPovLeft.whileTrue(drivetrain.applyRequest(
        //         () -> robotOrientedDrive
        //                 .withVelocityX(
        //                         align.getAlignForwardSpeedPercent(AlignConstants.REEF_ALIGN_TZ,
        // align.getReefAlignTag())
        //                                 * MaxSpeed)
        //                 .withVelocityY(align.getAlignStrafeSpeedPercent(
        //                                 AlignConstants.REEF_ALIGN_LEFT_TX, align.getReefAlignTag())
        //                         * MaxSpeed)
        //                 .withRotationalRate(align.getAlignRotationSpeedPercent(align.getAlignAngleReef())
        //                         * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //         ));

        // alignPovRight.whileTrue(drivetrain.applyRequest(() -> robotOrientedDrive
        //         .withVelocityX(align.getAlignForwardSpeedPercent(AlignConstants.REEF_ALIGN_TZ,
        // align.getReefAlignTag())
        //                 * MaxSpeed)
        //         .withVelocityY(
        //                 align.getAlignStrafeSpeedPercent(AlignConstants.REEF_ALIGN_RIGHT_TX, align.getReefAlignTag())
        //                         * MaxSpeed)
        //         .withRotationalRate(align.getAlignRotationSpeedPercent(align.getAlignAngleReef()) *
        // MaxAngularRate)));

        strafeTriggers.whileTrue(drivetrain.applyRequest(
                () -> robotOrientedDrive
                        .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y
                        .withVelocityY((driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis())
                                * 0.1
                                * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driverController.getRightX()
                                * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        elevatorAdjust.whileTrue(
                new RunCommand(() -> elevator.changeElevatorOffset(.01 * Math.signum(-opController.getLeftY()))));
        armAdjust.whileTrue(new RunCommand(() -> arm.armAdjust(.01 * Math.signum(opController.getRightX()))));

        resetHeading_Start.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        handoff_back.onTrue(new InstantCommand(() -> arm.setStowed())
                .andThen(new InstantCommand(() -> gIntake.setStowed()))
                .andThen(new InstantCommand(() -> endEffector.coralIn()))
                .until(() -> endEffector.hasCoral())
                .andThen(new InstantCommand(() -> gIntake.intake()))
                .until(() -> endEffector.hasCoral())
                .andThen(new InstantCommand(() -> endEffector.holdCoral())));

        intake_LB.onTrue(new InstantCommand(() -> endEffector.collectGamePiece()));

        levelFour_Y.onTrue(new InstantCommand(() -> elevator.setElevatorLevelFourMode())
                .andThen(new InstantCommand(() -> arm.setArmL4())));
        levelThree_B.onTrue(new InstantCommand(() -> elevator.setElevatorLevelThreeMode())
                .andThen(new InstantCommand(() -> arm.setArmL3())));
        levelTwo_X.onTrue(new InstantCommand(() -> elevator.setElevatorLevelTwoMode())
                .andThen(new InstantCommand(() -> arm.setArmL2())));
        station_Back.onTrue(new InstantCommand(() -> elevator.setElevatorStationMode())
                .andThen(new InstantCommand(() -> gIntake.setIntakePos()))
                .andThen(new InstantCommand(() -> arm.setArmIntake()))
                .andThen(new WaitCommand(0.3))
                .andThen(new InstantCommand(() -> gIntake.setStowed())));
        stow_A.onTrue(new InstantCommand(() -> elevator.setElevatorStowedMode())
                .andThen(new InstantCommand(() -> gIntake.setIntakePos()))
                .andThen(new InstantCommand(() -> arm.setStowed()))
                .andThen(new WaitCommand(0.3))
                .andThen(new InstantCommand(() -> gIntake.setStowed())));

        gIntakeDeploy_LT
                .onTrue(new InstantCommand(() -> gIntake.setIntakePos()))
                .onFalse(new InstantCommand(() -> gIntake.setStowed()));
        gIntakeShoot_RT
                .onTrue(new InstantCommand(() -> gIntake.setOuttakePos()))
                .onFalse(new InstantCommand(() -> gIntake.setStowed()));

        deployClimber_PovUp.whileTrue(new RunCommand(() -> climber.deployClimber()));
        retractClimber_PovDown.whileTrue(new RunCommand(() -> climber.retractClimber()));
        zeroClimber_PovLeft.onTrue(new InstantCommand(() -> climber.zeroClimber()));

        coralMode_LB.onTrue(new InstantCommand(() -> elevator.setCoral())
                .andThen(new InstantCommand(() -> arm.setCoral()))
                .andThen(new InstantCommand(() -> endEffector.setCoral()))
                .andThen(new InstantCommand(() -> gIntake.setCoralMode()))
                .andThen(new InstantCommand(() -> endEffector.stop()))
                .andThen(new InstantCommand(() -> ledstrip.setLEDsCoralMode())));
        algaeMode_RB.onTrue(new InstantCommand(() -> elevator.setAlgae())
                .andThen(new InstantCommand(() -> arm.setAlgae()))
                .andThen(new InstantCommand(() -> endEffector.setAlgae()))
                .andThen(new InstantCommand(() -> gIntake.setAlgaeMode()))
                .andThen(new InstantCommand(() -> ledstrip.setLEDsAlgaeMode())));

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
                        .andThen(new InstantCommand(() -> arm.setArmL4())));

        NamedCommands.registerCommand("Outtake", new InstantCommand(() -> endEffector.coralOut()));
        NamedCommands.registerCommand(
                "Intake", new InstantCommand(() -> endEffector.coralIn()).until(() -> endEffector.hasCoral()));
        NamedCommands.registerCommand("Stop EE", new InstantCommand(() -> endEffector.stopCoral()));
        NamedCommands.registerCommand("Hold Coral", new InstantCommand(() -> endEffector.holdCoral()));

        // NamedCommands.registerCommand(
        //         "Auto Align Left",
        //         drivetrain
        //                 .applyRequest(
        //                         () -> robotOrientedDrive
        //                                 .withVelocityX(align.getAlignForwardSpeedPercent(
        //                                                 AlignConstants.REEF_ALIGN_TZ, align.getReefAlignTag())
        //                                         * MaxSpeed)
        //                                 .withVelocityY(align.getAlignStrafeSpeedPercent(
        //                                                 AlignConstants.REEF_ALIGN_LEFT_TX, align.getReefAlignTag())
        //                                         * MaxSpeed)
        //                                 .withRotationalRate(align.getAlignRotationSpeedPercent(
        //                                                 align.getAlignAngleReef())
        //                                         * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //                         )
        //                 .withTimeout(1));

        // NamedCommands.registerCommand(
        //         "Auto Align Station",
        //         drivetrain.applyRequest(
        //                 () -> robotOrientedDrive
        //                         .withVelocityX(-align.getAlignForwardSpeedPercent(
        //                                         AlignConstants.STATION_ALIGN_TZ, align.getStationAlignTag())
        //                                 * MaxSpeed) // Drive forward with negative Y (forward)
        //                         .withVelocityY(-align.getAlignStrafeSpeedPercent(
        //                                         AlignConstants.STATION_ALIGN_TX, align.getStationAlignTag())
        //                                 * MaxSpeed) // Drive left with negative X (left)
        //                         .withRotationalRate(align.getAlignRotationSpeedPercent(align.getAlignAngleStation())
        //                                 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //                 ));
    }

    public void setDefaultCommands() {
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> drive.withVelocityX(drivetrain.frontLimiter.calculate(-driverController.getLeftY())
                                        * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(drivetrain.sideLimiter.calculate(-driverController.getLeftX())
                                        * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(drivetrain.turnLimiter.calculate(-driverController.getRightX())
                                        * MaxAngularRate) // Drive counterclockwise with negative X (left)
                        ));

        elevator.setDefaultCommand(new ElevatorHold());
        arm.setDefaultCommand(new ArmHold());
        climber.setDefaultCommand(new ClimbCommand());
        gIntake.setDefaultCommand(new GroundIntakeHold());
    }
}
