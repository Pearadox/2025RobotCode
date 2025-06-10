package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionRots = 0.0;
        public double velocityRps = 0.0;

        public double appliedVolts = 0.0;

        public double statorCurrent = 0.0;
        public double supplyCurrent = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void reachGoal(double setpoint) {}

    public default void setSpeed(double speed) {}

    public default void setPosition(double position) {}
}
