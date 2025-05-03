package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.SmarterDashboard;
import org.littletonrobotics.junction.Logger;

/** Class to run the rollers over CAN */
public class Climber extends SubsystemBase {
    // private final SparkMax climbMotor;
    private final PearadoxTalonFX climbMotor;

    private static final Climber CLIMBER = new Climber();
    private TalonFXConfiguration talonFXConfigs;

    private static final Arm ARM = Arm.getInstance();

    private double setpoint = ClimbConstants.CLIMB_SETPOINT;
    private double climberOffset = 0;

    private int climbState = 1;
    private String climbStateString = "unpowered";

    public static Climber getInstance() {
        return CLIMBER;
    }

    public Climber() {
        climbMotor = new PearadoxTalonFX(ClimbConstants.CLIMB_MOTOR_ID, NeutralModeValue.Brake, 50, false);

        BaseStatusSignal.setUpdateFrequencyForAll(
                ArmConstants.UPDATE_FREQ,
                climbMotor.getPosition(),
                climbMotor.getMotorVoltage(),
                climbMotor.getTorqueCurrent(),
                climbMotor.getSupplyCurrent(),
                climbMotor.getStatorCurrent());

        climbMotor.optimizeBusUtilization();

        talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = 0;
        slot0Configs.kS = 0;
        slot0Configs.kV = 0;
        slot0Configs.kA = 0;
        slot0Configs.kP = 0.25;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        climbMotor.getConfigurator().apply(slot0Configs);
    }

    @Override
    public void periodic() {

        updateClimStateString();

        SmarterDashboard.putNumber("Climber/Position", climbMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Climber/Pos", climbMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Climber/Volts", climbMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(
                "Climber/Supply Current", climbMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput(
                "Climber/Supply Current", climbMotor.getStatorCurrent().getValueAsDouble());

        SmarterDashboard.putNumber("Climber/State", climbState);
        SmarterDashboard.putString("Climber/StateStr", climbStateString);
    }

    public void climberAdjustUp() {
        climberOffset += 10; // in rotations
    }

    public void climberAdjustDown() {
        climberOffset -= 10;
    }

    // public void prepClimber() {
    //     if (climbState != 0) {
    //         ELEVATOR.setElevatorStowedMode();
    //         ARM.setAlgae();
    //         ARM.setStowed();
    //     }
    // }

    public void zeroClimber() {
        climbMotor.setPosition(0);
        climberOffset = 0;
    }

    public void stop() {
        climbMotor.set(0);
    }

    public void updateClimStateString() {
        if (climbState == 2) {
            climbStateString = "Climbing!";
        } else if (climbState == 1) {
            climbStateString = "Deployed (Vertical)";
        } else if (climbState == 0) {
            climbStateString = "Retracted (Stowed)";
        }
    }

    public void setClimbPosition() {
        if (climbState == 2) {
            climbClimber();
        } else if (climbState == 1) {
            deployClimber();
        } else if (climbState == 0) {
            retractClimber();
        }
    }

    public void retractClimber() {
        climbMotor.setControl(new PositionVoltage(-290 + climberOffset)); // -230 // -150 // -270
    }

    public void climberDown() {
        climbMotor.set(-0.8);
    }

    public void climberUp() {
        climbMotor.set(0.8);
    }

    public void deployClimber() {
        climbMotor.setControl(new PositionVoltage(0 + climberOffset));
    }

    public void climbClimber() {
        climbMotor.setControl(new PositionVoltage(-195 + climberOffset));
    }

    public void incrementClimbState() {
        climbState = Math.min(2, climbState + 1);
        // prepClimber();
        setClimbPosition();
    }

    public void decrementClimbState() {
        climbState = Math.max(0, climbState - 1);
        setClimbPosition();
    }
}
