package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorIOInputs {
        public double positionRots = 0.0;
        public double velocityRps = 0.0;

        public double appliedVolts = 0.0;

        public double statorCurrent = 0.0;
        public double supplyCurrent = 0.0;
    }

    public default void updateInputs(EndEffectorIOInputs inputs) {}

    public default void setSpeed(double speed) {}
}
