// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlignConstants;
import frc.robot.commands.ArmHold;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ElevatorHold;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LEDStrip;
import frc.robot.util.vision.PoseEstimation;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
    // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
    // second max angular velocity

    public static final Elevator elevator = Elevator.getInstance();
    public static final Arm arm = Arm.getInstance();
    public static final EndEffector endEffector = EndEffector.getInstance();
    public static final Climber climber = Climber.getInstance();
    public static final LEDStrip ledstrip = LEDStrip.getInstance();
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
    // motors
    private final SwerveRequest.RobotCentric robotOrientedDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
    // motors
    private final SwerveRequest.FieldCentricFacingAngle pointTowards = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
    // motors;
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controller
    public static final XboxController driverController = new XboxController(0);
    public static final XboxController opController = new XboxController(1);

    // Driver Controller
    private final JoystickButton resetHeading_Start =
            new JoystickButton(driverController, XboxController.Button.kStart.value);

    private final POVButton alignPovLeft = new POVButton(driverController, 270);
    private final POVButton alignPovDown = new POVButton(driverController, 180);
    private final POVButton alignPovRight = new POVButton(driverController, 90);
    private final POVButton alignPOVUp = new POVButton(driverController, 0);

    private final JoystickButton slowmode_A = new JoystickButton(driverController, XboxController.Button.kA.value);

    // Op Controller
    private final JoystickButton intake_LB = new JoystickButton(opController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton outtake_RB =
            new JoystickButton(opController, XboxController.Button.kRightBumper.value);

    private final JoystickButton homeElevator_Start =
            new JoystickButton(opController, XboxController.Button.kStart.value);

    private final JoystickButton tempCS_Back = new JoystickButton(opController, XboxController.Button.kBack.value);

    private final JoystickButton coralMode_LB =
            new JoystickButton(opController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton algaeMode_RB =
            new JoystickButton(opController, XboxController.Button.kRightBumper.value);

    private final JoystickButton levelFour_Y = new JoystickButton(opController, XboxController.Button.kY.value);
    private final JoystickButton levelThree_B = new JoystickButton(opController, XboxController.Button.kB.value);
    private final JoystickButton levelTwo_X = new JoystickButton(opController, XboxController.Button.kX.value);
    private final JoystickButton stow_A = new JoystickButton(opController, XboxController.Button.kA.value);

    private final POVButton zeroClimber_PovLeft = new POVButton(opController, 270);
    private final POVButton deployClimber_PovUp = new POVButton(opController, 0);
    private final POVButton retractClimber_PovDown = new POVButton(opController, 180);

    private final Trigger strafeTriggers = new Trigger(
            () -> Math.abs(driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis()) > 0.1);

    private final Trigger elevatorAdjust = new Trigger(() -> Math.abs(opController.getLeftY()) > 0.9);
    private final Trigger armAdjust = new Trigger(() -> Math.abs(opController.getRightX()) > 0.9);

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final SendableChooser<Command> autoChooser;

    private final AutoAlign align;

    public static final PoseEstimation poseEstimation = new PoseEstimation();

    public RobotContainer() {
        setDefaultCommands();
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("2R_EF-L4");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();

        align = new AutoAlign(() -> drivetrain.getState().Pose);

        new EventTrigger("LevelStation")
                .onTrue(new InstantCommand(() -> elevator.setElevatorStationMode())
                        .andThen(new InstantCommand(() -> arm.setArmIntake())));
        new EventTrigger("LevelFour")
                .onTrue(new InstantCommand(() -> elevator.setElevatorLevelFourMode())
                        .andThen(new InstantCommand(() -> arm.setArmL4())));
    }

    private void configureBindings() {
        homeElevator_Start
                .whileTrue(new InstantCommand(() -> elevator.setZeroing(true)))
                .onFalse(new InstantCommand(() -> elevator.stopElevator())
                        .withTimeout(0.2)
                        .andThen(new InstantCommand(() -> elevator.zeroElevator())
                                .andThen(new InstantCommand(() -> elevator.setZeroing(false)))));

        slowmode_A.onTrue(new InstantCommand(() -> drivetrain.changeSpeedMultiplier()));

        alignPOVUp.whileTrue(drivetrain
                .applyRequest(() -> robotOrientedDrive
                        .withVelocityX(-align.getAlignForwardSpeedPercent(
                                        AlignConstants.STATION_ALIGN_TZ, align.getStationAlignTag())
                                * MaxSpeed)
                        .withVelocityY(-align.getAlignStrafeSpeedPercent(
                                        AlignConstants.STATION_ALIGN_TX, align.getStationAlignTag())
                                * MaxSpeed)
                        .withRotationalRate(
                                align.getAlignRotationSpeedPercent(align.getAlignAngleStation()) * MaxAngularRate))
                .alongWith(ledstrip.aligning(() -> align.isAligned())));

        alignPovDown.whileTrue(drivetrain
                .applyRequest(() -> robotOrientedDrive
                        .withVelocityX(
                                align.getAlignForwardSpeedPercent(AlignConstants.REEF_ALIGN_TZ, align.getReefAlignTag())
                                        * MaxSpeed)
                        .withVelocityY(align.getAlignStrafeSpeedPercent(
                                        AlignConstants.REEF_ALIGN_MID_TX, align.getReefAlignTag())
                                * MaxSpeed)
                        .withRotationalRate(
                                align.getAlignRotationSpeedPercent(align.getAlignAngleReef()) * MaxAngularRate))
                .alongWith(ledstrip.aligning(() -> align.isAligned())));

        alignPovLeft.whileTrue(drivetrain
                .applyRequest(() -> robotOrientedDrive
                        .withVelocityX(
                                align.getAlignForwardSpeedPercent(AlignConstants.REEF_ALIGN_TZ, align.getReefAlignTag())
                                        * MaxSpeed)
                        .withVelocityY(align.getAlignStrafeSpeedPercent(
                                        AlignConstants.REEF_ALIGN_LEFT_TX, align.getReefAlignTag())
                                * MaxSpeed)
                        .withRotationalRate(
                                align.getAlignRotationSpeedPercent(align.getAlignAngleReef()) * MaxAngularRate))
                .alongWith(ledstrip.aligning(() -> align.isAligned())));

        alignPovRight.whileTrue(drivetrain
                .applyRequest(() -> robotOrientedDrive
                        .withVelocityX(
                                align.getAlignForwardSpeedPercent(AlignConstants.REEF_ALIGN_TZ, align.getReefAlignTag())
                                        * MaxSpeed)
                        .withVelocityY(align.getAlignStrafeSpeedPercent(
                                        AlignConstants.REEF_ALIGN_RIGHT_TX, align.getReefAlignTag())
                                * MaxSpeed)
                        .withRotationalRate(
                                align.getAlignRotationSpeedPercent(align.getAlignAngleReef()) * MaxAngularRate))
                .alongWith(ledstrip.aligning(() -> align.isAligned())));

        strafeTriggers.whileTrue(drivetrain.applyRequest(() -> robotOrientedDrive
                .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                .withVelocityY((driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis())
                        * 0.1
                        * MaxSpeed)
                .withRotationalRate(-driverController.getRightX() * MaxAngularRate)));

        elevatorAdjust.whileTrue(
                new RunCommand(() -> elevator.changeElevatorOffset(.01 * Math.signum(-opController.getLeftY()))));
        armAdjust.whileTrue(new RunCommand(() -> arm.armAdjust(.01 * Math.signum(opController.getRightX()))));

        resetHeading_Start.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        intake_LB.onTrue(new InstantCommand(() -> endEffector.collectGamePiece()));

        stow_A.onTrue(new InstantCommand(() -> elevator.setElevatorStowedMode())
                .andThen(new InstantCommand(() -> arm.setStowed())));
        tempCS_Back.onTrue(new InstantCommand(() -> elevator.setElevatorStationMode())
                .andThen(new InstantCommand(() -> arm.setArmIntake())));
        levelTwo_X.onTrue(new InstantCommand(() -> elevator.setElevatorLevelTwoMode())
                .andThen(new InstantCommand(() -> arm.setArmL2())));
        levelThree_B.onTrue(new InstantCommand(() -> elevator.setElevatorLevelThreeMode())
                .andThen(new InstantCommand(() -> arm.setArmL3())));
        levelFour_Y.onTrue(new InstantCommand(() -> elevator.setElevatorLevelFourMode())
                .andThen(new InstantCommand(() -> arm.setArmL4())));

        deployClimber_PovUp.whileTrue(new RunCommand(() -> climber.deployClimber()));
        retractClimber_PovDown.whileTrue(new RunCommand(() -> climber.retractClimber()));
        zeroClimber_PovLeft.onTrue(new InstantCommand(() -> climber.zeroClimber()));

        coralMode_LB.onTrue(new InstantCommand(() -> elevator.setCoral())
                .andThen(new InstantCommand(() -> arm.setCoral()))
                .andThen(new InstantCommand(() -> endEffector.setCoral()))
                .andThen(new InstantCommand(() -> endEffector.stop())));
        algaeMode_RB.onTrue(new InstantCommand(() -> elevator.setAlgae())
                .andThen(new InstantCommand(() -> arm.setAlgae()))
                .andThen(new InstantCommand(() -> endEffector.setAlgae())));

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
                        .andThen((drivetrain
                                .applyRequest(() -> robotOrientedDrive
                                        .withVelocityX(align.getAlignForwardSpeedPercent(
                                                        AlignConstants.REEF_ALIGN_TZ, align.getReefAlignTag())
                                                * MaxSpeed)
                                        .withVelocityY(align.getAlignStrafeSpeedPercent(
                                                        AlignConstants.REEF_ALIGN_LEFT_TX, align.getReefAlignTag())
                                                * MaxSpeed)
                                        .withRotationalRate(
                                                align.getAlignRotationSpeedPercent(align.getAlignAngleReef())
                                                        * MaxAngularRate))
                                .alongWith(ledstrip.aligning(() -> align.isAligned()))))
                        .withTimeout(1.4));

        NamedCommands.registerCommand(
                "Auto Align Right",
                new InstantCommand(() -> align.setReefAlignTagIDtoClosest())
                        .andThen((drivetrain
                                .applyRequest(() -> robotOrientedDrive
                                        .withVelocityX(align.getAlignForwardSpeedPercent(
                                                        AlignConstants.REEF_ALIGN_TZ, align.getReefAlignTag())
                                                * MaxSpeed)
                                        .withVelocityY(align.getAlignStrafeSpeedPercent(
                                                        AlignConstants.REEF_ALIGN_RIGHT_TX, align.getReefAlignTag())
                                                * MaxSpeed)
                                        .withRotationalRate(
                                                align.getAlignRotationSpeedPercent(align.getAlignAngleReef())
                                                        * MaxAngularRate))
                                .alongWith(ledstrip.aligning(() -> align.isAligned()))))
                        .withTimeout(1.4));

        NamedCommands.registerCommand(
                "Auto Align Station",
                (drivetrain
                                .applyRequest(() -> robotOrientedDrive
                                        .withVelocityX(-align.getAlignForwardSpeedPercent(
                                                        AlignConstants.STATION_ALIGN_TZ, align.getStationAlignTag())
                                                * MaxSpeed)
                                        .withVelocityY(-align.getAlignStrafeSpeedPercent(
                                                        AlignConstants.STATION_ALIGN_TX, align.getStationAlignTag())
                                                * MaxSpeed)
                                        .withRotationalRate(
                                                align.getAlignRotationSpeedPercent(align.getAlignAngleStation())
                                                        * MaxAngularRate))
                                .alongWith(ledstrip.aligning(() -> align.isAligned())))
                        .withTimeout(1.25));
    }

    public void setDefaultCommands() {
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> drive.withVelocityX(drivetrain.frontLimiter.calculate(-driverController.getLeftY())
                                        * MaxSpeed
                                        * drivetrain.getSpeedMultipler()) // Drive forward with negative Y (forward)
                                .withVelocityY(drivetrain.sideLimiter.calculate(-driverController.getLeftX())
                                        * MaxSpeed
                                        * drivetrain.getSpeedMultipler()) // Drive left with negative X (left)
                                .withRotationalRate(drivetrain.turnLimiter.calculate(-driverController.getRightX())
                                        * MaxAngularRate) // Drive counterclockwise with negative X (left)
                        ));

        elevator.setDefaultCommand(new ElevatorHold());
        arm.setDefaultCommand(new ArmHold());
        climber.setDefaultCommand(new ClimbCommand());
        ledstrip.setDefaultCommand(ledstrip.defaultCommand(() -> endEffector.isCoral()));
    }
}
