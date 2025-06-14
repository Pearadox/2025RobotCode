package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.Arrays;
import java.util.List;

public class Constants {
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class VisionConstants {
        public static final String LL_NAME = "limelight-back";
        public static final String LL_B_NAME = "limelight-front";

        public static final Vector<N3> LIMELIGHT_STD_DEV = VecBuilder.fill(.7, .7, .9999999);
        public static final Vector<N3> MEGATAG2_LIMELIGHT_STD_DEV = VecBuilder.fill(.7, .7, .9999999);

        public static final double AMBIGUITY_FILTER = 0.3;
        public static final double DISTANCE_FILTER = FieldConstants.FIELD_LENGTH / 2;

        // TODO: use andymark field layout for fit events
        public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    public static final class FieldConstants {
        public static final double FIELD_LENGTH = Units.inchesToMeters(690.876);
        public static final double FIELD_WIDTH = Units.inchesToMeters(317);

        public static final int[] BLUE_REEF_TAG_IDS = {18, 19, 20, 21, 22, 17};
        public static final int[] BLUE_CORAL_STATION_TAG_IDS = {12, 13};
        public static final int[] RED_REEF_TAG_IDS = {7, 6, 11, 10, 9, 8};
        public static final int[] RED_CORAL_STATION_TAG_IDS = {1, 2};

        public static final Translation2d BLUE_FAR_CAGE =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
        public static final Translation2d BLUE_MID_CAGE =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
        public static final Translation2d BLUE_CLOSE_CAGE =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

        public static final Translation2d RED_FAR_CAGE = new Translation2d(
                Units.inchesToMeters(345.428), Units.inchesToMeters(FIELD_WIDTH / 2 + (FIELD_WIDTH / 2 - 286.779)));
        public static final Translation2d RED_MID_CAGE = new Translation2d(
                Units.inchesToMeters(345.428), Units.inchesToMeters(FIELD_WIDTH / 2 + (FIELD_WIDTH / 2 - 242.855)));
        public static final Translation2d RED_CLOSE_CAGE = new Translation2d(
                Units.inchesToMeters(345.428), Units.inchesToMeters(FIELD_WIDTH / 2 + (FIELD_WIDTH / 2 - 199.947)));

        public static final Translation2d[] BLUE_CAGES = {BLUE_FAR_CAGE, BLUE_MID_CAGE, BLUE_CLOSE_CAGE};
        public static final Translation2d[] RED_CAGES = {RED_FAR_CAGE, RED_MID_CAGE, RED_CLOSE_CAGE};

        public static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.5, 4);

        public static final Translation2d BLUE_NPS_CORAL_STATION =
                new Translation2d(Units.inchesToMeters(33.526), Units.inchesToMeters(291.176));
        public static final Translation2d BLUE_PS_CORAL_STATION =
                new Translation2d(Units.inchesToMeters(33.526), Units.inchesToMeters(25.824));

        public static final Translation2d RED_NPS_CORAL_STATION =
                new Translation2d(FIELD_LENGTH - Units.inchesToMeters(33.526), Units.inchesToMeters(291.176));
        public static final Translation2d RED_PS_CORAL_STATION =
                new Translation2d(FIELD_LENGTH - Units.inchesToMeters(33.526), Units.inchesToMeters(25.824));

        public static final List<Pose2d> CORAL_STATIONS = Arrays.asList(
                new Pose2d(BLUE_NPS_CORAL_STATION, new Rotation2d(125)),
                new Pose2d(BLUE_PS_CORAL_STATION, new Rotation2d(-125)),
                new Pose2d(RED_NPS_CORAL_STATION, new Rotation2d(-125)),
                new Pose2d(RED_PS_CORAL_STATION, new Rotation2d(125)));

        // the top of the branch (L4) is ~2" behind the april tag
        public static final double BRANCH_OFFSET_BEHIND_APRILTAG = Units.inchesToMeters(2.049849);
        public static final double L4_HEIGHT = Units.inchesToMeters(72);

        public static final Pose3d[] REEF_TAG_POSES = new Pose3d[RED_REEF_TAG_IDS.length + BLUE_REEF_TAG_IDS.length];

        static {
            int i = 0;
            for (int tag : FieldConstants.RED_REEF_TAG_IDS) {
                REEF_TAG_POSES[i++] =
                        VisionConstants.aprilTagLayout.getTagPose(tag).get();
            }
            for (int tag : FieldConstants.BLUE_REEF_TAG_IDS) {
                REEF_TAG_POSES[i++] =
                        VisionConstants.aprilTagLayout.getTagPose(tag).get();
            }
        }

        public static final Transform3d HIGH_ALGAE_TRANSFORM =
                new Transform3d(Units.inchesToMeters(-6), 0, Units.inchesToMeters(39.575), Rotation3d.kZero);
        public static final Transform3d LOW_ALGAE_TRANSFORM =
                new Transform3d(Units.inchesToMeters(-6), 0, Units.inchesToMeters(23.675), Rotation3d.kZero);

        public static final Pose3d[] REEF_ALGAE_POSES = new Pose3d[REEF_TAG_POSES.length];

        static {
            for (int i = 0; i < REEF_ALGAE_POSES.length; i++) {
                REEF_ALGAE_POSES[i] = REEF_TAG_POSES[i].plus(i % 2 == 0 ? HIGH_ALGAE_TRANSFORM : LOW_ALGAE_TRANSFORM);
            }
        }

        public static final double BARGE_X = FIELD_LENGTH / 2.0;
        public static final double BARGE_WIDTH = Units.inchesToMeters(40) / 2.0;
        public static final double BARGE_HEIGHT = Units.inchesToMeters(74 + 8);
        public static final double BARGE_HEIGHT_TOLERANCE = Units.inchesToMeters(12);
        public static final double EE_TOLERANCE = Units.inchesToMeters(16);
    }

    public static final class DriveConstants {
        public static final double ROBOT_ORIENTED_TRIGGER_OFFSET = 0.4;

        public static final double DEFAULT_DEADBAND = 0.07;
        public static final double DRIVER_ALIGNING_DEADBAND = 0.15;

        public static final double SLOW_MODE_SPEED = 0.3;
    }

    public static final class AlignConstants {
        // TODO: possible align pid adjustment
        // public static final double ALIGN_STRAFE_KP = 0.06;
        // public static final double ALIGN_STRAFE_KI = 0.001;
        // public static final double ALIGN_FORWARD_KP = 0.04; // -0.06

        public static final double ALIGN_KS = 0.02; // 0.009

        // tx and ty tolerances with setpoint
        public static final double ALIGN_TOLERANCE_PIXELS = 0.5;
        // don't try translationally aligning unless rotation is already aligned within this tolerance
        public static final double ALIGN_ROT_TOLERANCE_DEGREES = 10;

        // reduce speed by 1/4 every tick when an april tag is not seen
        public static final double ALIGN_DAMPING_FACTOR = 0.75;
        public static final double ALIGN_SPEED_DEADBAND = 0.025;

        public static final double BRANCH_SPACING = Units.inchesToMeters(12.97 / 2.0); // 12.94 //12.97

        // target relative
        public static final double REEF_ALIGN_MID_TX = 0; // 0.28575
        public static final double REEF_ALIGN_LEFT_TX = -BRANCH_SPACING - 0.05 + 0.01;
        public static final double REEF_ALIGN_RIGHT_TX = BRANCH_SPACING - 0.03 + 0.01;
        public static final double REEF_ALIGN_TZ = Units.inchesToMeters(20);

        public static final double STATION_ALIGN_TX = 0.07;
        public static final double STATION_ALIGN_TZ = 0;

        public static final double REEF_kP = 0.5; // Tune all PID values
        public static final double REEF_kI = 0;
        public static final double REEF_kD = 0;

        public static final double REEF_Forward_kP = 0.2; // Tune all PID values

        // for some reason, 0.02 is much too low in sim??
        public static final double ROT_REEF_kP = Robot.isSimulation() ? Units.radiansToDegrees(0.02) : 0.02; 
        public static final double ROT_REEF_kI = 0;
        public static final double ROT_REEF_kD = 0;

        // the top of the branch (L4) is ~2" behind the april tag
        public static final double BRANCH_OFFSET_BEHIND_APRILTAG = Units.inchesToMeters(2.049849);

        // real heights of branches
        public static final double L4_HEIGHT = Units.inchesToMeters(72);
        public static final double L3_HEIGHT = Units.inchesToMeters(48);
        public static final double L2_HEIGHT = Units.inchesToMeters(36);

        // these are from the arm's pivot point to the bottom of a held coral
        public static final double PIVOT_TO_CORAL_RADIUS = Units.inchesToMeters(23.4106654653);
        public static final double ARM_TO_CORAL_ANGULAR_OFFSET = Units.degreesToRadians(34.8693502919);

        public static final double ARM_STARTING_ANGLE = Units.degreesToRadians(-96);
        // from the arm's pivot point to floor
        public static final double ELEVATOR_STARTING_HEIGHT = 1.0023799104; // Units.inchesToMeters(39);
    }

    public static final class ElevatorConstants {
        public static final int ELEVATOR_ID = 21;
        public static final int ELEVATOR_FOLLOWER_ID = 20;
        public static final NeutralModeValue MODE = NeutralModeValue.Brake;
        public static final int CURRENT_LIMIT = 60; //
        public static final boolean IS_INVERTED = false;

        public static final int UPDATE_FREQ = 50;

        public static final double MM_CRUISE_VELCOCITY_UP = 100;
        public static final double MM_ACCELERATION_UP = 200;

        public static final double MM_CRUISE_VELCOCITY_DOWN = 1;
        public static final double MM_ACCELERATION_DOWN = 1;

        public static final double TICKS_PER_REV = 4000;
        public static final double GEAR_RATIO = 3;
        public static final double PULLEY_DIAMETER = 2.005;
        public static final double kRotationToInches = PULLEY_DIAMETER * Math.PI / GEAR_RATIO;

        // Inches
        public static final double STOWED_HEIGHT = 0;
        public static final double STATION_HEIGHT =
                15; // Home Field is 1in higher than official field - official is 15.4
        public static final double LEVEL_TWO_HEIGHT =
                0; // 10.9; // was 12, 7 This is slightly away from the reef for clearance //
        public static final double LEVEL_THREE_HEIGHT = 1.5; // 15 //TODO l3 height
        public static final double LEVEL_FOUR_HEIGHT = 24.6; // 29.625; //

        public static final double ALGAE_LOW_HEIGHT = 6.4; // 6.7
        public static final double ALGAE_HIGH_HEIGHT = 16;

        public static final double BARGE_HEIGHT = 32; // 33

        public static final double MAX_ELEVATOR_HEIGHT = 34;

        public static final double HOMING_SPEED = -0.05;

        public static final double LEVEL_TWO_ROT = LEVEL_TWO_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double STATION_ROT = STATION_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double OBSTRUCTED_STATION_ROT =
                STATION_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER) - 1.26;
        public static final double LEVEL_THREE_ROT = LEVEL_THREE_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double LEVEL_FOUR_ROT = LEVEL_FOUR_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double ALGAE_LOW_ROT = ALGAE_LOW_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double ALGAE_HIGH_ROT = ALGAE_HIGH_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double MAX_ELEVATOR_ROT = MAX_ELEVATOR_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double BARGE_ROT = BARGE_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double STOWED_ROT = STOWED_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);

        public static final double ELEVATOR_OFFSET = 0.075;

        public static final double kG = 0.29; // 0.3
        public static final double kS = 0.11; // 0
        public static final double kV = 0.1; // 0
        public static final double kA = 0.0; // 0
        public static final double kP = 0.4; // 0.4
        public static final double kI = 0.0; // 0
        public static final double kD = 0.0; // 0
    }

    public static final class ArmConstants {
        public static final int ARM_KRAKEN_ID = 22;
        public static final int CURRENT_LIMIT = 20;

        public static final double ARM_GEAR_RATIO = 60; // ?

        public static final double ARM_LENGTH = 0; // TODO

        public static final double ARM_LEVEL_4_ROT =
                Units.degreesToRotations(156) * ARM_GEAR_RATIO - 0.52; // -170 //-180 //0155.94 //-161
        public static final double ARM_LEVEL_3_ROT =
                Units.degreesToRotations(147) * ARM_GEAR_RATIO - 0.3 - 0.19; // was -78 //79.08 // -66

        public static final double ARM_LEVEL_2_ROT =
                Units.degreesToRotations(147) * ARM_GEAR_RATIO - 6.382; // was -85, then -74.455078125[\] //79.08
        public static final double ARM_INTAKE_ROT =
                Units.degreesToRotations(-15) * ARM_GEAR_RATIO; //  was 61 //-299 // -311
        public static final double ARM_STOWED_ROT = Units.degreesToRotations(0) * ARM_GEAR_RATIO; // should be 0

        public static final double ARM_ALGAE_LOW = Units.degreesToRotations(73) * ARM_GEAR_RATIO;
        public static final double ARM_ALGAE_HIGH = Units.degreesToRotations(82) * ARM_GEAR_RATIO;
        public static final double ARM_BARGE = Units.degreesToRotations(160) * ARM_GEAR_RATIO - 2.1;
        // public static final double ARM_CLIMB = Units.degreesToRotations(-50) * ARM_GEAR_RATIO;
        public static final double ARM_LOLLIPOP = Units.degreesToRotations(-50) * ARM_GEAR_RATIO;
        public static final double ARM_L4_BEHIND_CORAL = 22.057; // rots
        public static final double ARM_STATION_BEHIND_CORAL = Units.degreesToRotations(-15) * ARM_GEAR_RATIO - 1.63;

        public static final double ARM_ADJUST_INCREMENT = 0.1;

        public static final double UPDATE_FREQ = 50;

        public static final double kG = 0.0; // 0.35;
        public static final double kS = 0.15; // 0.15;
        public static final double kV = 7.2; // 0.2
        public static final double kA = 0.0; // 0.07
        public static final double kP = 0.4; // 0.3
        public static final double kI = 0.0;
        public static final double kD = 0.0; // 0.05

        public static final double MM_MAX_CRUISE_VELOCITY = 2;
        public static final double MM_MAX_CRUISE_ACCELERATION = 0.5;
    }

    public static final class ClimbConstants {
        public static final int CLIMB_MOTOR_ID = 24;
        public static final int CLIMB_MOTOR_CURRENT_LIMIT = 40;
        public static final double CLIMB_MOTOR_VOLTAGE_COMP = 10;
        public static final double CLIMB_VALUE = 0.7;

        public static final double CLIMB_ROTS = -126.0;
        public static final double CLIMB_SETPOINT = -210.0;

        public static final double CLIMBER_LENGTH = Units.inchesToMeters(8);
        public static final double MASS_KG = Units.lbsToKilograms(3);
        public static final double MOI = MASS_KG * CLIMBER_LENGTH * CLIMBER_LENGTH; // most of the mass is the barb
        public static final double GEAR_RATIO = 290.0 * 4.0;
        public static final double MIN_ANGLE = Units.degreesToRadians(0);
        public static final double MAX_ANGLE = Units.degreesToRadians(90);
        public static final double STARTING_ANGLE = MIN_ANGLE;
        public static final double ZERO_ANGLE = MAX_ANGLE;
    }

    public static final class IntakeConstants {
        public static final int PIVOT_ID = 40;

        // public static final int PIVOT_GEAR_RATIO = 60;
        public static final NeutralModeValue MODE = NeutralModeValue.Brake;
        public static final int ROLLER_CURRENT_LIMIT = 30;
        public static final int PIVOT_CURRENT_LIMIT = 30;

        // PID for pivot
        public static final double PIVOT_kP = 0.5;
        public static final double PIVOT_kI = 0.0;
        public static final double PIVOT_kD = 0.0;

        public static final double PIVOT_MIN_OUTPUT = -0.5;
        public static final double PIVOT_MAX_OUTPUT = 0.5;

        public static final double PIVOT_GEARING = 60; // 25:1 reduction gear ratio

        public static final double PIVOT_OUTTAKE_ROT =
                Units.degreesToRotations(10.0) * PIVOT_GEARING; // TODO: find outtake rot in motor rotations
        public static final double PIVOT_INTAKE_ROT =
                Units.degreesToRotations(105.0) * PIVOT_GEARING; // TODO: find intake rot in motor rotations
        public static final double PIVOT_STOWED_ROT =
                Units.degreesToRotations(0.0) * PIVOT_GEARING; // TODO: find stowed rot in motor rotations
        public static final double PIVOT_ALGAE_ROT =
                Units.degreesToRotations(40.0) * PIVOT_GEARING; // TODO: find stowed rot in motor rotations

        // CAN ID for roller
        public static final int ROLLER_ID = 41;
    }

    public static final class EndEffectorConstants {
        public static final int END_EFFECTOR_ID = 23;

        public static final double PULL_SPEED = -0.3;

        public static final double PUSH_SPEED = 0.6; // 0.3
        public static final double L3_PUSH_SPEED = 0.3; // 0.3
        public static final double L2_PUSH_SPEED = 0.1; // 0.3
        public static final double ALGAE_PULL_SPEED = 0.8;
        public static final double ALGAE_PUSH_SPEED = -1.0;

        public static final double HOLD_SPEED = -0.075;

        public static final int END_SENSOR_CHANNEL = 0;
    }

    public static final class LEDConstants {
        public static final int NUM_LEDS = 28;
        public static final int PORT = 9;

        // to adjust
        public static final Frequency SCROLL_FREQ = Percent.per(Second).of(50);
        public static final Time BLINK_PERIOD = Seconds.of(0.5);
        public static final Time BLINKING_DURATION = BLINK_PERIOD.times(2);
        public static final Dimensionless BLINK_BRIGHTNESS = Percent.of(50);
        public static final Time BREATHE_PERIOD = Seconds.of(1);
    }

    public static class SimulationConstants {
        public static final boolean SIMULATE_GRAVITY = true;

        public static final double ARM_MASS = Units.lbsToKilograms(8);
        public static final double ARM_LENGTH = Units.inchesToMeters(12);
        public static final double ARM_MOI = SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS);
        public static final double MIN_ANGLE = Double.NEGATIVE_INFINITY;
        public static final double MAX_ANGLE = Double.POSITIVE_INFINITY;
        public static final double STARTING_ANGLE = Units.degreesToRadians(-96);

        // joint of ee to bottom of coral
        public static final double CAM_LENGTH = Units.inchesToMeters(14.5);

        // joint of ee to top plane of coral
        public static final double EE_TO_CORAL_HEIGHT = Units.inchesToMeters(2.5);
        public static final double EE_TO_CORAL_WIDTH = Units.inchesToMeters(4.25);
        public static final double CORAL_LENGTH = Units.inchesToMeters(11.875);

        public static final double CARRIAGE_MASS_KG = 4.2;
        public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(ElevatorConstants.PULLEY_DIAMETER / 2.0);
        public static final double MIN_HEIGHT = Units.inchesToMeters(0);
        public static final double MAX_HEIGHT = MIN_HEIGHT + Units.inchesToMeters(ElevatorConstants.BARGE_HEIGHT + 1);
        public static final double STARTING_HEIGHT = MIN_HEIGHT;

        // new EE viz
        public static final double PIVOT_TO_MIDDLE_OF_CORAL_ANG_OFFSET = Units.degreesToRadians(20.2531597269);
        public static final double PIVOT_TO_MIDDLE_OF_CORAL_RADIUS = Units.inchesToMeters(23.249031544);
        public static final double PIVOT_ANGLE_TO_CORAL_ANGLE = Units.degreesToRadians(-243.986 + 15);
        public static final double CORAL_Y_OFFSET = 0.02;

        public static final double ARM_CAD_ZERO_Z = AlignConstants.ELEVATOR_STARTING_HEIGHT;
        public static final double CLIMBER_CAD_ZERO_Y = Units.inchesToMeters(13.5);
        public static final double CLIMBER_CAD_ZERO_Z = Units.inchesToMeters(9);
        public static final double CLIMBER_CAD_ANG_OFFSET = Units.degreesToRadians(65);
    }
}
