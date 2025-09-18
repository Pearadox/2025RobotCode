package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double positionRots = 0.0;
        public double velocityRps = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrent = 0.0;
        public double supplyCurrent = 0.0;
    }

    default void updateInputs(ClimberIOInputs inputs) {}

    default void runPosition(double setpoint) {}

    default void setSpeed(double speed) {}

    default void zeroPosition() {}
}
