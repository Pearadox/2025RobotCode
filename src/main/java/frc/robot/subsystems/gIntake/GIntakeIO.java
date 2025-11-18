package frc.robot.subsystems.gIntake;

import org.littletonrobotics.junction.AutoLog;

public interface GIntakeIO {
    @AutoLog // makes functions to automatically log stuff you put in the function under
    public static class GIntakeIOInputs {
        // put anything you want to log in here
        public double pivotPositionRots = 0.0;
        public double pivotSpeedRps = 0.0;

        public double pivotStatorCurrent = 0.0;
        public double pivotSupplyCurrent = 0.0;
        // stator current is the MOTOR->OUTPUT current - controls TORQUE
        // supply current is the BATTERY->MOTOR current - controls POWER

        public double pivotMotorVoltage = 0.0;

        public double rollerPositionRots = 0.0; // not really a practical use for this besides sim
        public double rollerSpeedRps = 0.0;
        public double rollerVoltage = 0.0;
    }

    public void updateInputs(GIntakeIOInputsAutoLogged GIntakeIOInputs);

    public void runPivotPosition(double setpoint);

    public void runPivotVoltage(double voltage);

    public void runRollerVoltage(double voltage);
}
