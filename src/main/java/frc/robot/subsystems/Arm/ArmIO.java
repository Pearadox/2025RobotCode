package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double positionRots = 0.0;
        public double velocityRps = 0.0;

        public double appliedVolts = 0.0;

        public double statorCurrent = 0.0;
        public double supplyCurrent = 0.0;
    }

    public default void updateInputs(ArmIOInputsAutoLogged inputs) {}

    public default void runPosition(double setpoint, double feedforward) {}
}
