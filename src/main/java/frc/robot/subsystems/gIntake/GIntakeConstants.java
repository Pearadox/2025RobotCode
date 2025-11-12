package frc.robot.subsystems.gIntake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.util.Units;

public class GIntakeConstants {
    public static enum GIntakeState { // NAME(pivot angle in degrees, roller speed)
        STOWED(0.0, 0.0),
        INTAKE(105.0, 1.0),
        OUTTAKE(10.0, -1.0),
        ALGAE(40.0, -1.0);

        private double setpoint;
        private double speed;

        private GIntakeState(double pivot_deg, double roller_speed) {
            setpoint = Units.degreesToRotations(pivot_deg) * PIVOT_GEARING;
            speed = roller_speed;
        }

        public double getPivotSetpoint() {
            return setpoint;
        }

        public double getRollerSpeed() {
            return speed;
        }

        public void adjustSetpoint(double rotations) {
            setpoint += PIVOT_GEARING * rotations;
        }
    }

    public static final int PIVOT_ID = 40;
    public static final int ROLLER_ID = 41;

    public static final int PIVOT_CURRENT_LIMIT = 50;
    public static final int ROLLER_CURRENT_LIMIT = 30;

    public static final int PIVOT_GEARING = 0;
    public static final int ROLLER_GEARING = 0;

    public static final TalonFXConfiguration PIVOT_CONFIGS = new TalonFXConfiguration();
    public static final Slot0Configs SLOT_0_CONFIGS = PIVOT_CONFIGS.Slot0;
    
    public static final Slot0Configs getConfig() {
        SLOT_0_CONFIGS.kG = 0.0;
        SLOT_0_CONFIGS.kS = 0.0;
        SLOT_0_CONFIGS.kV = 0.0;
        SLOT_0_CONFIGS.kA = 0.0;
        SLOT_0_CONFIGS.kP = 0.0;
        SLOT_0_CONFIGS.kI = 0.0;
        SLOT_0_CONFIGS.kD = 0.0;
        return SLOT_0_CONFIGS;
    }


}
