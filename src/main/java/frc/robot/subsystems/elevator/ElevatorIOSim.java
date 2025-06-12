package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SimulationConstants;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(2),
            ElevatorConstants.GEAR_RATIO,
            SimulationConstants.CARRIAGE_MASS_KG,
            SimulationConstants.DRUM_RADIUS_METERS,
            SimulationConstants.MIN_HEIGHT,
            SimulationConstants.MAX_HEIGHT,
            SimulationConstants.SIMULATE_GRAVITY,
            SimulationConstants.STARTING_HEIGHT);

    private PearadoxTalonFX elevatorMotor;
    private TalonFXSimState elevatorMotorSimState;
    private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    public ElevatorIOSim() {
        elevatorMotor = new PearadoxTalonFX(
                ElevatorConstants.ELEVATOR_ID,
                ElevatorConstants.MODE,
                ElevatorConstants.CURRENT_LIMIT,
                !ElevatorConstants.IS_INVERTED);

        elevatorMotorSimState = elevatorMotor.getSimState();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = ElevatorConstants.kG;
        slot0Configs.kS = ElevatorConstants.kS;
        slot0Configs.kV = ElevatorConstants.kV;
        slot0Configs.kA = ElevatorConstants.kA;
        slot0Configs.kP = ElevatorConstants.kP;
        slot0Configs.kI = ElevatorConstants.kI;
        slot0Configs.kD = ElevatorConstants.kD;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity =
                ElevatorConstants.MM_CRUISE_VELCOCITY_UP; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration =
                ElevatorConstants.MM_ACCELERATION_UP; // Target acceleration of 160 rps/s (0.5 seconds)
        // (not sure if needed - > ) motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1
        // seconds)

        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                33 * ElevatorConstants.GEAR_RATIO / (Math.PI * ElevatorConstants.PULLEY_DIAMETER);
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        talonFXConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        elevatorMotor.getConfigurator().apply(talonFXConfigs);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        updateSim();

        inputs.positionRots = elevatorMotor.getPosition().getValueAsDouble();
        inputs.velocityRps = elevatorMotor.getVelocity().getValueAsDouble();

        inputs.appliedVolts = elevatorMotor.getMotorVoltage().getValueAsDouble();

        inputs.statorCurrent = elevatorMotor.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrent = elevatorMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void reachGoal(double setpoint) {
        elevatorMotor.setControl(motionMagicRequest.withPosition(setpoint));
    }

    @Override
    public void setSpeed(double speed) {
        elevatorMotor.set(speed);
    }

    @Override
    public void setPosition(double position) {
        elevatorMotor.setPosition(position);
    }

    private void updateSim() {
        elevatorMotorSimState.setSupplyVoltage(12);

        elevatorSim.setInput(elevatorMotorSimState.getMotorVoltage());
        elevatorSim.update(0.02);

        elevatorMotorSimState.setRawRotorPosition(getMotorRotations(elevatorSim.getPositionMeters()));

        // angular velocity = linear velocity / radius
        elevatorMotorSimState.setRotorVelocity(
                ((elevatorSim.getVelocityMetersPerSecond() / SimulationConstants.DRUM_RADIUS_METERS)
                                // radians/sec to rotations/sec
                                / (2.0 * Math.PI))
                        * ElevatorConstants.GEAR_RATIO);

        // MechVisualizer.getInstance().updateElevatorHeight(elevatorSim.getPositionMeters());
    }

    private static double getMotorRotations(double linearDisplacement) {
        // angular displacement in radians = linear displacement / radius
        return Units.radiansToRotations(linearDisplacement / SimulationConstants.DRUM_RADIUS_METERS)
                // multiply by gear ratio
                * ElevatorConstants.GEAR_RATIO;
    }
}
