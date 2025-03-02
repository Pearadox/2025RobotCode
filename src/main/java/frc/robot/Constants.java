// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static class ElevatorConstants {

        public static final int ELEVATOR_ID = 21; // TODO
        public static final int ELEVATOR_FOLLOWER_ID = 20; // TODO
        public static final NeutralModeValue MODE = NeutralModeValue.Brake;
        public static final int CURRENT_LIMIT = 60;
        public static final boolean IS_INVERTED = false;

        public static final int UPDATE_FREQ = 50;

        public static final double MM_CRUISE_VELCOCITY = 45; // TODO
        public static final double MM_ACCELERATION = 20; // TODO

        public static final double TICKS_PER_REV = 4000;
        public static final double GEAR_RATIO = 3;
        public static final double PULLEY_DIAMETER = 2.005;
        public static final double kRotationToInches = PULLEY_DIAMETER * Math.PI / GEAR_RATIO;

        // the following are in inches
        public static final double STOWED_HEIGHT = 0;
        public static final double STATION_HEIGHT = 1.3; // TODO
        public static final double LEVEL_TWO_HEIGHT =
                9.1; // was 7 This is slightly away from the reef for clearance //TODO
        public static final double LEVEL_THREE_HEIGHT = 25; // TODO was 21[]\
        public static final double LEVEL_FOUR_HEIGHT = 30; // TODO
        public static final double MAX_ELEVATOR_HEIGHT = 30; // TODO

        public static final double LEVEL_TWO_ROT = LEVEL_TWO_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double STATION_ROT = STATION_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double LEVEL_THREE_ROT = LEVEL_THREE_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double LEVEL_FOUR_ROT = LEVEL_FOUR_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double MAX_ELEVATOR_ROT = MAX_ELEVATOR_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);
        public static final double STOWED_ROT = STOWED_HEIGHT * GEAR_RATIO / (Math.PI * PULLEY_DIAMETER);

        public static final double ELEVATOR_OFFSET = 0.0;

        // TODO: change all of these values to match true elevator gains
        public static final double kG = 0.245;
        public static final double kS = 0.;
        public static final double kV = 0.;
        public static final double kA = 0.0;
        public static final double kP = 0.6;
        public static final double kI = 0.00;
        public static final double kD = 0.000;
    }
}
