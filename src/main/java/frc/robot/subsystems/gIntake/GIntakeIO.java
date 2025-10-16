package frc.robot.subsystems.gIntake;

import org.littletonrobotics.junction.AutoLog;

public interface GIntakeIO {
    @AutoLog // makes functions to automatically log stuff you put in the function under
    public static class GIntakeIOInputs {
        // put anything you want to log in here
        public double positionRots = 0;
        public double rollerSpeedRps = 0;

        public double pivotStatorCurrent = 0;
        public double pivotSupplyCurrent = 0;
        // stator current is the MOTOR->OUTPUT current - controls TORQUE
        // supply current is the BATTERY->MOTOR current - controls POWER
    }

    public void updateInputs(GIntakeIOInputsAutoLogged GIntakeIOInputs);

    public void runPosition(double setpoint, boolean isIntaking, double feedforward);
}
