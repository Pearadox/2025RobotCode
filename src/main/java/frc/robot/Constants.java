package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class ElevatorConstants {

        public static final int ELEVATOR_ID = 21;
        public static final int ELEVATOR_FOLLOWER_ID = 20;
        public static final NeutralModeValue MODE = NeutralModeValue.Brake;
        public static final int CURRENT_LIMIT = 60; // TODO
        public static final boolean IS_INVERTED = false;

        public static final int UPDATE_FREQ = 50;

        public static final double MAX_VELOCITY_MPS = 2.0; // TODO
        public static final double MAX_ACCELERATION_MPS2 = 8.0; // TODO
        public static final double MM_CRUISE_VELCOCITY = 45; // TODO
        public static final double MM_ACCELERATION = 35; // TODO

        public static final double TICKS_PER_REV = 4000; // TODO
        public static final double GEAR_RATIO = 3; // TODO
        public static final double PULLEY_DIAMETER = 2.005; // TODO should be chain pitch * number of teeth / pi
        public static final double kRotationToInches = PULLEY_DIAMETER * Math.PI / GEAR_RATIO;

        // the following are in inches
        public static final double STOWED_HEIGHT = 0;
        public static final double STATION_HEIGHT = 4.625; // TODO
        public static final double LEVEL_TWO_HEIGHT =
                8.625; // 10.9; // was 12, 7 This is slightly away from the reef for clearance //TODO
        public static final double LEVEL_THREE_HEIGHT = 18.000; // 24.495; //TODO
        public static final double LEVEL_FOUR_HEIGHT = 25.000; // 29.625; //TODO

        public static final double MAX_ELEVATOR_HEIGHT = 20; // TODO

        public static final double LEVEL_TWO_ROT = LEVEL_TWO_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double STATION_ROT = STATION_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double LEVEL_THREE_ROT = LEVEL_THREE_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double LEVEL_FOUR_ROT = LEVEL_FOUR_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double MAX_ELEVATOR_ROT = MAX_ELEVATOR_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double STOWED_ROT = STOWED_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);

        public static final double ELEVATOR_OFFSET = 0.05;

        // TODO: change all of these values to match true elevator gains
        public static final double kG = 0.0; // 0.245
        public static final double kS = 0.; // 0.2
        public static final double kV = 0.; // 0.3
        public static final double kA = 0.0; // 0.02
        public static final double kP = 0.0; // 0.6
        public static final double kI = 0.00; // 0
        public static final double kD = 0.000; // 0
    }

    public static final class ArmConstants {
        // TODO: Update all constants for arm
        public static final int ARM_KRAKEN_ID = 22;
        public static final int CURRENT_LIMIT = 40;

        public static final double ARM_GEAR_RATIO = 60; // TODO?

        public static final double ARM_LEVEL_4_ROT = Units.degreesToRotations(-165); // -170 //-180 //0155.94 //-161
        public static final double ARM_LEVEL_3_ROT = Units.degreesToRotations(-69.5); // was -78 //79.08 // -66
        public static final double ARM_LEVEL_2_ROT =
                Units.degreesToRotations(-66); // was -85, then -74.455078125[\] //79.08
        public static final double ARM_INTAKE_ROT = Units.degreesToRotations(-307); //  was 61 //-299 // -311
        public static final double ARM_STOWED_ROT = Units.degreesToRotations(0); // should be 0

        public static final double ARM_ADJUST_INCREMENT = 0.075;

        public static final double UPDATE_FREQ = 50;

        // TODO: tune pid
        public static final double kG = 0.0; // 0.15
        public static final double kS = 0.0;
        public static final double kV = 0.0; // 0.1
        public static final double kA = 0.0;
        public static final double kP = 0.25; // 0.2
        public static final double kI = 0.0; // 0.05
        public static final double kD = 0.04;
    }

    public static final class EndEffectorConstants {
        public static final int END_EFFECTOR_ID = 23;

        public static final double PULL_SPEED = -1;
        public static final double PUSH_SPEED = 0.5;

        public static final int END_SENSOR_CHANNEL = 0;
    }
}
