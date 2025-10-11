package frc.robot.subsystems.gIntake;

import org.littletonrobotics.junction.AutoLog;

public interface GIntakeIO {
    @AutoLog // makes functions to automatically log stuff you put in the function under
    public static class GIntakeIOInputs {
        // put anything you want to log in here
        public double positionRots = 0;
        public double rollerSpeedRots = 0;

        // public double P = 0;
        // public double I = 0;
        // public double D = 0; // logging PID good for initial troubleshooting I guess

        public double pivotCurrent = 0; // whats the difference between stator and supply current?
    }

    public void updateInputs(GIntakeIOInputsAutoLogged GIntakeIOInputs);

    public void runPosition(double setpoint, boolean isIntaking, double feedforward);
}
