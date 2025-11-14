package frc.robot.subsystems.gIntake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class GIntakeConstants {
    public static enum GIntakeState { // NAME(pivot angle in degrees, roller speed)
        CORAL_STOWED(0.0, 0.0),
        CORAL_INTAKE(105.0, 1.0),
        CORAL_OUTTAKE(10.0, -1.0),
        ALGAE_STOWED(40.0, 0.0),
        ALGAE_INTAKE(40.0, -1.0),
        ALGAE_OUTTAKE(40.0, 1.0);

        private double setpoint;
        private double adjust;
        private double speed;

        private GIntakeState(double pivot_deg, double roller_speed) {
            setpoint = Units.degreesToRotations(pivot_deg) * PIVOT_GEARING;
            adjust = 0;
            speed = roller_speed;
        }

        public double getPivotSetpoint() {
            return setpoint + adjust;
        }

        public double getRollerSpeed() {
            return speed;
        }

        public double getAdjust() {
            return adjust;
        }

        public void adjustSetpoint(double rotations) {
            adjust += PIVOT_GEARING * rotations;
        }

        public void resetAdjust() {
            adjust = 0;
        }

        public String toString() {
            switch (this) {
                case CORAL_STOWED:
                    return "CORAL_STOWED";
                case CORAL_INTAKE:
                    return "CORAL_INTAKE";
                case CORAL_OUTTAKE:
                    return "CORAL_OUTTAKE";
                case ALGAE_STOWED:
                    return "ALGAE_STOWED";
                case ALGAE_INTAKE:
                    return "ALGAE_INTAKE";
                case ALGAE_OUTTAKE:
                    return "ALGAE_OUTTAKE";
                default:
                    return null;
            }
        }
    }

    public static final int PIVOT_ID = 40;
    public static final int ROLLER_ID = 41;

    public static final int PIVOT_CURRENT_LIMIT = 50;
    public static final int ROLLER_CURRENT_LIMIT = 30;

    public static final int PIVOT_GEARING = 60;
    public static final int ROLLER_GEARING = 0;

    public static final TalonFXConfiguration PIVOT_CONFIGS = new TalonFXConfiguration();
    public static final Slot0Configs SLOT_0_CONFIGS = PIVOT_CONFIGS.Slot0;

    public static final Slot0Configs getConfig() {
        SLOT_0_CONFIGS.kG = 0.0;
        SLOT_0_CONFIGS.kS = 0.0;
        SLOT_0_CONFIGS.kV = 0.0;
        SLOT_0_CONFIGS.kA = 0.0;
        SLOT_0_CONFIGS.kP = 1.0;
        SLOT_0_CONFIGS.kI = 0.0;
        SLOT_0_CONFIGS.kD = 0.0;
        return SLOT_0_CONFIGS;
    }

    // sim
    public static final double GINTAKE_MASS = Units.lbsToKilograms(10);
    public static final double GINTAKE_LENGTH = Units.inchesToMeters(16);
    public static final double GINTAKE_MOI = SingleJointedArmSim.estimateMOI(GINTAKE_LENGTH, GINTAKE_MASS);
    public static final double GINTAKE_STARTING_ANGLE = Units.degreesToRadians(90);
}
