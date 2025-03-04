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
        public static final double LEVEL_THREE_HEIGHT = 24.495; // 26.769; //TODO was 21
        public static final double LEVEL_FOUR_HEIGHT = 29.625; // 32.718; //TODO

        public static final double MAX_ELEVATOR_HEIGHT = 20; // TODO

        public static final double LEVEL_TWO_ROT = LEVEL_TWO_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double STATION_ROT = STATION_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double LEVEL_THREE_ROT = LEVEL_THREE_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double LEVEL_FOUR_ROT = LEVEL_FOUR_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double MAX_ELEVATOR_ROT = MAX_ELEVATOR_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double STOWED_ROT = STOWED_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);

        public static final double ELEVATOR_OFFSET = 0.05;

        // TODO: change all of these values to match true elevator gains
        public static final double kG = 0.245; //
        public static final double kS = 0.; // 0.2
        public static final double kV = 0.; // 0.3
        public static final double kA = 0.0; // 0.02
        public static final double kP = 0.6; // 0.4
        public static final double kI = 0.00; // 0
        public static final double kD = 0.000; // 0
    }
    
}
