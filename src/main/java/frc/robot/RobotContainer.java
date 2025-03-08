// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.Align;
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
import frc.robot.util.vision.PoseEstimation;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final Elevator elevator = Elevator.getInstance();
    public static final Arm arm = Arm.getInstance();
    public static final EndEffector endEffector = EndEffector.getInstance();
    public static final Climber climbSubsystem = new Climber();

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
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors;
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controller
    private static final XboxController driverController = new XboxController(0);
    public static final XboxController opController = new XboxController(1);

    private final JoystickButton resetHeading_Start =
            new JoystickButton(driverController, XboxController.Button.kStart.value);
    private final JoystickButton elevatorUp_Y = new JoystickButton(driverController, XboxController.Button.kY.value);
    private final JoystickButton elevatorDown_A = new JoystickButton(driverController, XboxController.Button.kA.value);
    private final JoystickButton armAdjustUp_X = new JoystickButton(driverController, XboxController.Button.kX.value);
    private final JoystickButton armAdjustDown_B = new JoystickButton(driverController, XboxController.Button.kB.value);
    // private final Trigger strafeTriggers = new Trigger(() -> (Math.abs(driverController.getRightTriggerAxis() -
    // driverController.getLeftTriggerAxis()) > 0.1));
    private final POVButton alignPovLeft = new POVButton(driverController, 270);
    private final POVButton alignPovDown = new POVButton(driverController, 180);
    private final POVButton alignPovRight = new POVButton(driverController, 90);
    private final JoystickButton slewLeftBumper =
            new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton robotOrientedRightBumper =
            new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);

    private final POVButton climberUp = new POVButton(opController, 0);
    private final POVButton climberDown = new POVButton(opController, 180);
    private final POVButton resetAdjusts = new POVButton(opController, 270);

    private final JoystickButton levelFour_Y = new JoystickButton(opController, XboxController.Button.kY.value);
    private final JoystickButton levelThree_B = new JoystickButton(opController, XboxController.Button.kB.value);
    private final JoystickButton levelTwo_X = new JoystickButton(opController, XboxController.Button.kX.value);
    private final JoystickButton stow_A = new JoystickButton(opController, XboxController.Button.kA.value);
    private final JoystickButton station_Start = new JoystickButton(opController, XboxController.Button.kStart.value);
    private final JoystickButton barge_Back = new JoystickButton(opController, XboxController.Button.kBack.value);
    private final JoystickButton coralAlgaeSwap_LB =
            new JoystickButton(opController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton climb_RB = new JoystickButton(opController, XboxController.Button.kRightBumper.value);

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final SendableChooser<Command> autoChooser;

    private final AutoAlign align;

    public static final PoseEstimation poseEstimation = new PoseEstimation();

    public RobotContainer() {
        setDefaultCommands();
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("GH_L4");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();

        align = new AutoAlign(() -> drivetrain.getState().Pose);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
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

        slewLeftBumper.whileTrue(drivetrain.applyRequest(
                () -> drive.withVelocityX(drivetrain.frontLimiter.calculate(-driverController.getLeftY())
                                * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(drivetrain.sideLimiter.calculate(-driverController.getLeftX())
                                * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(drivetrain.turnLimiter.calculate(-driverController.getRightX())
                                * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        robotOrientedRightBumper.whileTrue(drivetrain.applyRequest(
                () -> robotOrientedDrive
                        .withVelocityX(drivetrain.frontLimiter.calculate(-driverController.getLeftY())
                                * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(drivetrain.sideLimiter.calculate(-driverController.getLeftX())
                                * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(drivetrain.turnLimiter.calculate(-driverController.getRightX())
                                * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        coralAlgaeSwap_LB.onTrue(new InstantCommand(() -> elevator.changeIsCoral())
                .andThen(new InstantCommand(() -> arm.changeIsCoral())));

        // strafeTriggers.whileTrue(
        //     drivetrain.applyRequest(
        //         () -> robotOrientedDrive
        //             .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y
        // (forward)
        //             .withVelocityY(
        //                     (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis())
        //                     * 0.5 * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise
        // with negative X (left)
        //     )
        // );

        // alignPovLeft.whileTrue(drivetrain.applyRequest(() -> pointTowards
        //         .withVelocityX(align.getFieldRelativeChassisSpeeds(
        //                         AlignConstants.REEF_ALIGN_LEFT_TX,
        //                         AlignConstants.REEF_ALIGN_TY,
        //                         drivetrain.getState().Pose.getRotation(),
        //                         MaxSpeed)
        //                 .vxMetersPerSecond)
        //         .withVelocityY(align.getFieldRelativeChassisSpeeds(
        //                         AlignConstants.REEF_ALIGN_LEFT_TX,
        //                         AlignConstants.REEF_ALIGN_TY,
        //                         drivetrain.getState().Pose.getRotation(),
        //                         MaxSpeed)
        //                 .vyMetersPerSecond)
        //        .withTargetDirection(align.getAlignAngleReef())));

        // alignPovDown.whileTrue(drivetrain.applyRequest(() -> pointTowards
        //         .withVelocityX(align.getFieldRelativeChassisSpeeds(
        //                         AlignConstants.REEF_ALIGN_MID_TX,
        //                         AlignConstants.REEF_ALIGN_TY,
        //                         drivetrain.getState().Pose.getRotation(),
        //                         MaxSpeed)
        //                 .vxMetersPerSecond)
        //         .withVelocityY(align.getFieldRelativeChassisSpeeds(
        //                         AlignConstants.REEF_ALIGN_MID_TX,
        //                         AlignConstants.REEF_ALIGN_TY,
        //                         drivetrain.getState().Pose.getRotation(),
        //                         MaxSpeed)
        //                 .vyMetersPerSecond)
        //         .withTargetDirection(align.getAlignAngleReef())));

        // alignPovRight.whileTrue(drivetrain.applyRequest(() -> pointTowards
        //         .withVelocityX(align.getFieldRelativeChassisSpeeds(
        //                         AlignConstants.REEF_ALIGN_RIGHT_TX,
        //                         AlignConstants.REEF_ALIGN_TY,
        //                         drivetrain.getState().Pose.getRotation(),
        //                         MaxSpeed)
        //                 .vxMetersPerSecond)
        //         .withVelocityY(align.getFieldRelativeChassisSpeeds(
        //                         AlignConstants.REEF_ALIGN_RIGHT_TX,
        //                         AlignConstants.REEF_ALIGN_TY,
        //                         drivetrain.getState().Pose.getRotation(),
        //                         MaxSpeed)
        //                 .vyMetersPerSecond)
        //         .withTargetDirection(align.getAlignAngleReef())));

        // invertRobotOrientation.onTrue(new InstantCommand(() -> drivetrain.changeRobotOrientation()));

        elevator.setDefaultCommand(new ElevatorHold());
        arm.setDefaultCommand(new ArmHold());

        resetHeading_Start.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        elevatorUp_Y.whileTrue(new RunCommand(() -> elevator.changeElevatorOffset(ElevatorConstants.ELEVATOR_OFFSET)));
        elevatorDown_A.whileTrue(
                new RunCommand(() -> elevator.changeElevatorOffset(-ElevatorConstants.ELEVATOR_OFFSET)));
        armAdjustUp_X.whileTrue(new RunCommand(() -> arm.armAdjust(ArmConstants.ARM_ADJUST_INCREMENT)));
        armAdjustDown_B.whileTrue(new RunCommand(() -> arm.armAdjust(-ArmConstants.ARM_ADJUST_INCREMENT)));

        stow_A.onTrue(new InstantCommand(() -> elevator.setElevatorStowedMode())
                .andThen(new InstantCommand(() -> arm.setStowed())));
        station_Start.onTrue(new InstantCommand(() -> elevator.setElevatorStationMode())
                .andThen(new WaitCommand(0.5))
                .andThen(new InstantCommand(() -> arm.setArmIntake())));
        levelTwo_X.onTrue(new InstantCommand(() -> elevator.setElevatorLevelTwoMode())
                .andThen(new InstantCommand(() -> arm.setArmL2())));
        levelThree_B.onTrue(new InstantCommand(() -> elevator.setElevatorLevelThreeMode())
                .andThen(new InstantCommand(() -> arm.setArmL3())));
        levelFour_Y.onTrue(new InstantCommand(() -> elevator.setElevatorLevelFourMode())
                .andThen(new InstantCommand(() -> arm.setArmL4())));
        barge_Back.onTrue(new InstantCommand(() -> elevator.setElevatorBarge())
                .andThen(new InstantCommand(() -> arm.setBarge())));

        climberUp.whileTrue(new ClimbCommand(() -> ClimbConstants.CLIMB_VALUE, () -> 0, climbSubsystem));

        climberDown.whileTrue(new ClimbCommand(() -> -ClimbConstants.CLIMB_VALUE, () -> 0, climbSubsystem));

        resetAdjusts.onTrue(
                new InstantCommand(() -> elevator.resetAdjust()).andThen(new InstantCommand(() -> arm.resetAdjust())));

        climb_RB.onTrue(new InstantCommand(() -> arm.setClimb())
                .andThen(new InstantCommand(() -> elevator.setElevatorStowedMode())));
        // elevator.setElevatorStowedMode())));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // operatorController.start().onTrue(new InstantCommand(() -> elevator.setElevatorStowedMode()));
        // operatorController.x().onTrue(new InstantCommand(() -> elevator.setElevatorStationMode()));
        // operatorController.y().onTrue(new InstantCommand(() -> elevator.setElevatorLevelTwoMode()));
        // operatorController.b().onTrue(new InstantCommand(() -> elevator.setElevatorLevelThreeMode()));
        // operatorController.a().onTrue(new InstantCommand(() -> elevator.setElevatorLevelFourMode()));

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

        NamedCommands.registerCommand("Outtake", new InstantCommand(() -> endEffector.coralIn()));
        NamedCommands.registerCommand("Intake", new InstantCommand(() -> endEffector.coralOut()));
        NamedCommands.registerCommand("Stop EE", new InstantCommand(() -> endEffector.stopCoral()));

        NamedCommands.registerCommand(
                "Intake Station",
                new InstantCommand(() -> endEffector.coralIn())
                        .until(() -> endEffector.hasCoral())
                        .andThen(new InstantCommand(() -> endEffector.stop())));
    }

    public void setDefaultCommands() {
        // elevator.setDefaultCommand(new ElevatorHold());
        // arm.setDefaultCommand(new ArmHold());
    }
}
