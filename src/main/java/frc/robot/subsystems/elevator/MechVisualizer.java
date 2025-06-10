package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.SimulationConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class MechVisualizer {
    private final double elevatorAngle = 90; // straight up

    private final LoggedMechanism2d mech2d = new LoggedMechanism2d(Units.inchesToMeters(45), Units.inchesToMeters(90));
    private final LoggedMechanismRoot2d elevatorRoot = mech2d.getRoot("Elevator Root", Units.inchesToMeters(22.5), 0);
    private final LoggedMechanismLigament2d elevator2d = elevatorRoot.append(
            new LoggedMechanismLigament2d("Elevator", SimulationConstants.MIN_HEIGHT, elevatorAngle));
    private LoggedMechanismLigament2d arm = elevator2d.append(
            new LoggedMechanismLigament2d("Arm", Units.inchesToMeters(16.5), 0, 5, new Color8Bit(Color.kYellow)));

    private LoggedMechanismRoot2d climbRoot = mech2d.getRoot(
            "Climb Root",
            Units.inchesToMeters(22.5) + SimulationConstants.CLIMBER_CAD_ZERO_Y,
            SimulationConstants.CLIMBER_CAD_ZERO_Z);
    private LoggedMechanismLigament2d climber = climbRoot.append(new LoggedMechanismLigament2d(
            "Climber",
            ClimbConstants.CLIMBER_LENGTH,
            ClimbConstants.STARTING_ANGLE,
            3,
            new Color8Bit(Color.kLightSteelBlue)));

    // End effector segments (derived from CAD)
    private LoggedMechanismLigament2d ee23a = arm.append(new LoggedMechanismLigament2d(
            "EE23a", Units.inchesToMeters(5.6102), 180 - 109.295, 7, new Color8Bit(Color.kDarkSeaGreen)));
    private LoggedMechanismLigament2d ee34a = ee23a.append(new LoggedMechanismLigament2d(
            "EE34a", Units.inchesToMeters(2), -(180 - 163.914), 7, new Color8Bit(Color.kMediumSeaGreen)));
    private LoggedMechanismLigament2d ee45a1 = ee34a.append(new LoggedMechanismLigament2d(
            "EE45a1", Units.inchesToMeters(5.1181 / 2), -(180 - 151.395), 7, new Color8Bit(Color.kLightSeaGreen)));
    private LoggedMechanismLigament2d ee45a2 = ee45a1.append(new LoggedMechanismLigament2d(
            "EE45a2", Units.inchesToMeters(5.1181 / 2), 0, 7, new Color8Bit(Color.kPaleGreen)));

    private LoggedMechanismLigament2d ee23b = arm.append(new LoggedMechanismLigament2d(
            "EE23b", Units.inchesToMeters(5.6102), 180 + 109.295, 7, new Color8Bit(Color.kDarkBlue)));
    private LoggedMechanismLigament2d ee34b = ee23b.append(new LoggedMechanismLigament2d(
            "EE34b", Units.inchesToMeters(2), (180 - 163.914), 7, new Color8Bit(Color.kRoyalBlue)));
    private LoggedMechanismLigament2d ee45b1 = ee34b.append(new LoggedMechanismLigament2d(
            "EE45b1", Units.inchesToMeters(5.1181 / 2), (180 - 151.395), 7, new Color8Bit(Color.kDeepSkyBlue)));
    private LoggedMechanismLigament2d ee45b2 = ee45b1.append(new LoggedMechanismLigament2d(
            "EE45b2", Units.inchesToMeters(5.1181 / 2), 0, 7, new Color8Bit(Color.kSkyBlue)));

    private LoggedMechanismLigament2d coral1a = ee45a1.append(new LoggedMechanismLigament2d(
            "CORAL1a", SimulationConstants.CORAL_LENGTH / 2.0, 90, 8, new Color8Bit(Color.kPapayaWhip)));
    private LoggedMechanismLigament2d coral2a = ee45a1.append(new LoggedMechanismLigament2d(
            "CORAL2a", SimulationConstants.CORAL_LENGTH / 2.0, -90, 8, new Color8Bit(Color.kAntiqueWhite)));

    private double heightMeters = 0.0;
    private double armAngleRads = 0.0;
    private double climbAngRads = 0.0;

    private static final MechVisualizer instance = new MechVisualizer();

    public static MechVisualizer getInstance() {
        return instance;
    }

    private MechVisualizer() {}

    public void periodic() {
        SmartDashboard.putData("Elevator Sim", mech2d);
        Logger.recordOutput("FieldSimulation/Mechanism Visualizer", mech2d);

        Logger.recordOutput("FieldSimulation/Components", new Transform3d[] {
            new Transform3d(0, 0, heightMeters, new Rotation3d()),
            new Transform3d(
                    0,
                    0,
                    SimulationConstants.ARM_CAD_ZERO_Z + heightMeters,
                    new Rotation3d(0, -armAngleRads + SimulationConstants.STARTING_ANGLE, 0)),
            new Transform3d(
                    0,
                    SimulationConstants.CLIMBER_CAD_ZERO_Y,
                    SimulationConstants.CLIMBER_CAD_ZERO_Z,
                    new Rotation3d(-climbAngRads + SimulationConstants.CLIMBER_CAD_ANG_OFFSET, 0, 0))
        });
    }

    public void updateElevatorHeight(double heightMeters) {
        this.heightMeters = heightMeters;
        elevator2d.setLength(heightMeters + AlignConstants.ELEVATOR_STARTING_HEIGHT);
    }

    public void updateArmAngle(double angleRads) {
        this.armAngleRads = angleRads;
        arm.setAngle(Units.radiansToDegrees(angleRads) - elevatorAngle);
    }

    public void updateClimberRoll(double angleRads) {
        this.climbAngRads = angleRads;
        climber.setAngle(180 - Units.radiansToDegrees(climbAngRads));
    }
}
