package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorIOInput {
        boolean motorConnected = false;
        boolean followerConnected = false;
        double positionInches = 0.0;
        double velocityInchesPerSec = 0.0;
        double[] appliedVolts = new double[] {};
        double[] torqueCurrentAmps = new double[] {};
        double[] supplyCurrentAmps = new double[] {};
        double[] tempCelsius = new double[] {};
    }

    public default void updateInputs(ElevatorIOInputAutoLogged inputs) {}

    public default void setVoltage(double volts) {}

    public default void setPosition(double inches) {}

    public default void setControlConstants(
            double kG, double kS, double kV, double kA, double kP, double kI, double kD) {}

    public default void setMotionMagicConstants(double velocity, double acceleration) {}
}
