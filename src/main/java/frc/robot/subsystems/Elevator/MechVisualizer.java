package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class MechVisualizer {
    private final double elevatorAngle = 90; // straight up

    private final Mechanism2d mech2d = new Mechanism2d(Units.inchesToMeters(45), Units.inchesToMeters(90));
    private final MechanismRoot2d elevatorRoot = mech2d.getRoot("Elevator Root", Units.inchesToMeters(22.5), 0);
    private final MechanismLigament2d elevator2d =
            elevatorRoot.append(new MechanismLigament2d("Elevator", SimulationConstants.MIN_HEIGHT, elevatorAngle));
    private MechanismLigament2d arm = elevator2d.append(
            new MechanismLigament2d("Arm", Units.inchesToMeters(16.5), 0, 5, new Color8Bit(Color.kYellow)));

    // CAM
    // private MechanismLigament2d ee_A = arm.append(new MechanismLigament2d(
    //         "EE1", SimulationConstants.EE_TO_CORAL_HEIGHT, 0, 3, new Color8Bit(Color.kLightSeaGreen)));
    // private MechanismLigament2d ee_B = ee_A.append(new MechanismLigament2d(
    //         "EE2", SimulationConstants.EE_TO_CORAL_WIDTH, 0, 3, new Color8Bit(Color.kMediumSpringGreen)));
    // private MechanismLigament2d coral = ee_B.append(
    //         new MechanismLigament2d("CORAL", SimulationConstants.CORAL_LENGTH, 0, 2, new
    // Color8Bit(Color.kPapayaWhip)));

    private MechanismLigament2d ee23a = arm.append(new MechanismLigament2d(
            "EE23a", Units.inchesToMeters(5.6102), 180 - 109.295, 7, new Color8Bit(Color.kDarkSeaGreen)));
    private MechanismLigament2d ee34a = ee23a.append(new MechanismLigament2d(
            "EE34a", Units.inchesToMeters(2), -(180 - 163.914), 7, new Color8Bit(Color.kMediumSeaGreen)));
    private MechanismLigament2d ee45a1 = ee34a.append(new MechanismLigament2d(
            "EE45a1", Units.inchesToMeters(5.1181 / 2), -(180 - 151.395), 7, new Color8Bit(Color.kLightSeaGreen)));
    private MechanismLigament2d ee45a2 = ee45a1.append(
            new MechanismLigament2d("EE45a2", Units.inchesToMeters(5.1181 / 2), 0, 7, new Color8Bit(Color.kPaleGreen)));

    private MechanismLigament2d ee23b = arm.append(new MechanismLigament2d(
            "EE23b", Units.inchesToMeters(5.6102), 180 + 109.295, 7, new Color8Bit(Color.kDarkBlue)));
    private MechanismLigament2d ee34b = ee23b.append(new MechanismLigament2d(
            "EE34b", Units.inchesToMeters(2), (180 - 163.914), 7, new Color8Bit(Color.kRoyalBlue)));
    private MechanismLigament2d ee45b1 = ee34b.append(new MechanismLigament2d(
            "EE45b1", Units.inchesToMeters(5.1181 / 2), (180 - 151.395), 7, new Color8Bit(Color.kDeepSkyBlue)));
    private MechanismLigament2d ee45b2 = ee45b1.append(
            new MechanismLigament2d("EE45b2", Units.inchesToMeters(5.1181 / 2), 0, 7, new Color8Bit(Color.kSkyBlue)));

    // private MechanismLigament2d coral1 = ee45b1.append(new MechanismLigament2d(
    //         "CORAL1", SimulationConstants.CORAL_LENGTH / 2, 90, 10, new Color8Bit(Color.kPapayaWhip)));
    // private MechanismLigament2d coral2 = ee45b1.append(new MechanismLigament2d(
    //         "CORAL2", SimulationConstants.CORAL_LENGTH / 2, -90, 10, new Color8Bit(Color.kAntiqueWhite)));

    private MechanismLigament2d coral1a = ee45a1.append(new MechanismLigament2d(
            "CORAL1a", SimulationConstants.CORAL_LENGTH / 2.0, 90, 8, new Color8Bit(Color.kPapayaWhip)));
    private MechanismLigament2d coral2a = ee45a1.append(new MechanismLigament2d(
            "CORAL2a", SimulationConstants.CORAL_LENGTH / 2.0, -90, 8, new Color8Bit(Color.kAntiqueWhite)));

    // private MechanismLigament2d coral3 = coral2.append(new MechanismLigament2d(
    //         "CORAL3", Units.inchesToMeters(23.4106654653), 180 - 278.855350292, 1, new Color8Bit(Color.kCyan)));
    // private MechanismLigament2d coral4 = ee45b1.append(new MechanismLigament2d(
    //         "CORAL4", Units.inchesToMeters(23.249031544), 180 - 354.239159727, 1, new Color8Bit(Color.kRed)));

    // private MechanismLigament2d coral3a = coral1a.append(new MechanismLigament2d(
    //         "CORAL3a", Units.inchesToMeters(23.4106654653), 180 + 278.855350292, 1, new Color8Bit(Color.kCyan)));
    // private MechanismLigament2d coral4a = ee45a1.append(new MechanismLigament2d(
    //         "CORAL4a", Units.inchesToMeters(23.249031544), 180 + 354.239159727, 1, new Color8Bit(Color.kRed)));

    // private InterpolatingDoubleTreeMap camLerp = new InterpolatingDoubleTreeMap();

    private double heightMeters = 0.0;
    private double armAngleRads = 0.0;

    private static final MechVisualizer instance = new MechVisualizer();

    public static MechVisualizer getInstance() {
        return instance;
    }

    private MechVisualizer() {
        // camLerp.put(ArmState.L4.angle, -65.0);
        // camLerp.put(ArmState.L3.angle, -55.0);
        // camLerp.put(ArmState.L2.angle, -55.0);
        // camLerp.put(ArmState.CoralStation.angle, -180 - 35.0);
        // camLerp.put(-90.0, -90.0);
    }

    public void periodic() {
        SmartDashboard.putData("Elevator Sim", mech2d);

        Logger.recordOutput("Sim/Origin", new Pose3d());

        double armX = RobotContainer.driverController.getRightTriggerAxis();
        double armY = 0.342 + RobotContainer.driverController.getRightY();
        double armZ = 0.229 + RobotContainer.driverController.getLeftTriggerAxis(); // TODO: lower
        double armP = RobotContainer.driverController.getRightX() * 2 * Math.PI;

        Logger.recordOutput("Sim/Arm", new double[] {armX, armY, armZ, 0, armP, 0});

        Logger.recordOutput("Sim/Transforms", new Transform3d[] {
            new Transform3d(0, 0, heightMeters, new Rotation3d()),
            new Transform3d(
                    0,
                    0,
                    1.003 + heightMeters,
                    new Rotation3d(0, -armAngleRads + SimulationConstants.STARTING_ANGLE, 0)),
            new Transform3d(armX, armY, armZ, new Rotation3d(armP, 0, 0))
        });
    }

    public void updateElevatorHeight(double heightMeters) {
        this.heightMeters = heightMeters;
        elevator2d.setLength(heightMeters + AlignConstants.ELEVATOR_STARTING_HEIGHT);
        // ProjectileIntakeSim.getInstance().updateElevatorHeight(heightMeters);
    }

    public void updateArmAngle(double angleRads) {
        this.armAngleRads = angleRads;
        arm.setAngle(Units.radiansToDegrees(angleRads) - elevatorAngle);

        // double camAngle = camLerp.get(angleDegs);
        // ee_A.setAngle(camAngle - angleDegs); // parallel with coral
        // ee_B.setAngle(90); // perpendicular to coral
        // coral.setAngle(-90); // perpendicular to ee_B

        // ProjectileIntakeSim.getInstance().updateArmAngle(angleRads);
        // ProjectileIntakeSim.getInstance().updateCamAngle(Units.degreesToRadians(camAngle));
    }
}
