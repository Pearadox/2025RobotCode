package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.AlignConstants;
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

    // private LoggedMechanismLigament2d coral1 = ee45b1.append(new LoggedMechanismLigament2d(
    //         "CORAL1", SimulationConstants.CORAL_LENGTH / 2, 90, 10, new Color8Bit(Color.kPapayaWhip)));
    // private LoggedMechanismLigament2d coral2 = ee45b1.append(new LoggedMechanismLigament2d(
    //         "CORAL2", SimulationConstants.CORAL_LENGTH / 2, -90, 10, new Color8Bit(Color.kAntiqueWhite)));

    private LoggedMechanismLigament2d coral1a = ee45a1.append(new LoggedMechanismLigament2d(
            "CORAL1a", SimulationConstants.CORAL_LENGTH / 2.0, 90, 8, new Color8Bit(Color.kPapayaWhip)));
    private LoggedMechanismLigament2d coral2a = ee45a1.append(new LoggedMechanismLigament2d(
            "CORAL2a", SimulationConstants.CORAL_LENGTH / 2.0, -90, 8, new Color8Bit(Color.kAntiqueWhite)));

    // private LoggedMechanismLigament2d coral3 = coral2.append(new LoggedMechanismLigament2d(
    //         "CORAL3", Units.inchesToMeters(23.4106654653), 180 - 278.855350292, 1, new Color8Bit(Color.kCyan)));
    // private LoggedMechanismLigament2d coral4 = ee45b1.append(new LoggedMechanismLigament2d(
    //         "CORAL4", Units.inchesToMeters(23.249031544), 180 - 354.239159727, 1, new Color8Bit(Color.kRed)));

    // private LoggedMechanismLigament2d coral3a = coral1a.append(new LoggedMechanismLigament2d(
    //         "CORAL3a", Units.inchesToMeters(23.4106654653), 180 + 278.855350292, 1, new Color8Bit(Color.kCyan)));
    // private LoggedMechanismLigament2d coral4a = ee45a1.append(new LoggedMechanismLigament2d(
    //         "CORAL4a", Units.inchesToMeters(23.249031544), 180 + 354.239159727, 1, new Color8Bit(Color.kRed)));

    private double heightMeters = 0.0;
    private double armAngleRads = 0.0;

    private static final MechVisualizer instance = new MechVisualizer();

    public static MechVisualizer getInstance() {
        return instance;
    }

    private MechVisualizer() {}

    public void periodic() {
        SmartDashboard.putData("Elevator Sim", mech2d);
        Logger.recordOutput("FieldSimulation/Mechanism Visualizer", mech2d);

        double climberRoll = 0; // Math.sin(System.currentTimeMillis() / 1000.0) * Math.PI * 2;

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
                    new Rotation3d(climberRoll, 0, 0))
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
}
