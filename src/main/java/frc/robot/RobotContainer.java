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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ArmHold;
import frc.robot.commands.ElevatorHold;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final Elevator elevator = Elevator.getInstance();
    public static final Arm arm = Arm.getInstance();
    public static final EndEffector endEffector = EndEffector.getInstance();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controller
    private static final XboxController driverController = new XboxController(0);
    public static final XboxController opController = new XboxController(1);

    private final JoystickButton resetHeading_Start =
            new JoystickButton(driverController, XboxController.Button.kStart.value);
    private final JoystickButton elevatorUp = new JoystickButton(driverController, XboxController.Button.kY.value);
    private final JoystickButton elevatorDown = new JoystickButton(driverController, XboxController.Button.kA.value);
    private final JoystickButton armAdjustUp = new JoystickButton(driverController, XboxController.Button.kX.value);
    private final JoystickButton armAdjustDown = new JoystickButton(driverController, XboxController.Button.kB.value);
    private final JoystickButton setPID = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton climberUp = new JoystickButton(driverController, XboxController.Button.kBack.value);
    private final JoystickButton slowMode =
            new JoystickButton(driverController, XboxController.Button.kRightBumper.value);

    private final JoystickButton levelFour = new JoystickButton(opController, XboxController.Button.kY.value);
    private final JoystickButton levelThree = new JoystickButton(opController, XboxController.Button.kB.value);
    private final JoystickButton levelTwo = new JoystickButton(opController, XboxController.Button.kX.value);
    private final JoystickButton stow = new JoystickButton(opController, XboxController.Button.kA.value);
    private final JoystickButton station = new JoystickButton(opController, XboxController.Button.kStart.value);
    private final JoystickButton intake = new JoystickButton(opController, XboxController.Button.kBack.value);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        setDefaultCommands();
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("GH_L4");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                driverController.getLeftBumperButtonPressed() ?
                drivetrain.applyRequest(
                        () -> drive.withVelocityX(drivetrain.frontLimiter.calculate(-driverController.getLeftY())
                                        * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(
                                        drivetrain.sideLimiter.calculate(-driverController.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(drivetrain.turnLimiter.calculate(-driverController.getRightX())
                                        * MaxAngularRate) // Drive counterclockwise with negative X (left)
                        ):
               drivetrain.applyRequest(
                        () -> drive.withVelocityX(-driverController.getLeftY()
                                        * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(
                                        -driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(-driverController.getRightX()
                                        * MaxAngularRate) // Drive counterclockwise with negative X (left)
                        )
                );

        elevator.setDefaultCommand(new ElevatorHold());
        arm.setDefaultCommand(new ArmHold());

        resetHeading_Start.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        elevatorUp.whileTrue(new RunCommand(() -> elevator.changeElevatorOffset(ElevatorConstants.ELEVATOR_OFFSET)));
        elevatorDown.whileTrue(new RunCommand(() -> elevator.changeElevatorOffset(-ElevatorConstants.ELEVATOR_OFFSET)));
        armAdjustUp.whileTrue(new RunCommand(() -> arm.armAdjust(ArmConstants.ARM_ADJUST_INCREMENT)));
        armAdjustDown.whileTrue(new RunCommand(() -> arm.armAdjust(-ArmConstants.ARM_ADJUST_INCREMENT)));
        stow.onTrue(new InstantCommand(() -> elevator.setElevatorStowedMode())
                .andThen(new InstantCommand(() -> arm.setStowed())));
        station.onTrue(new InstantCommand(() -> elevator.setElevatorStationMode())
                .andThen(new WaitCommand(0.5))
                .andThen(new InstantCommand(() -> arm.setArmIntake())));
        levelTwo.onTrue(new InstantCommand(() -> elevator.setElevatorAlgaeLow())
                .andThen(new InstantCommand(() -> arm.setAlgaeLow())));
        levelThree.onTrue(new InstantCommand(() -> elevator.setElevatorLevelThreeMode())
                .andThen(new InstantCommand(() -> arm.setArmL3())));
        levelFour.onTrue(new InstantCommand(() -> elevator.setElevatorLevelFourMode())
                .andThen(new InstantCommand(() -> arm.setArmL4())));
        intake.onTrue(new InstantCommand(() -> elevator.setElevatorBarge())
                .andThen(new InstantCommand(() -> arm.setBarge())));

        setPID.onTrue(new InstantCommand(() -> elevator.setPID()));

        // elevator.setDefaultCommand(ElevatorCommands.setElevatorStowedMode(elevator));

        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController
        //         .b()
        //         .whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(
        //                 new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

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

        // // reset the field-centric heading on left bumper press
        // driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

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
    }

    public void setDefaultCommands() {
        // elevator.setDefaultCommand(new ElevatorHold());
        // arm.setDefaultCommand(new ArmHold());
    }
}
