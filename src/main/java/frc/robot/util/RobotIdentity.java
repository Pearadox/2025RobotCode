package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.generated.Eve2TunerConstants;
import frc.robot.generated.PearracudaTunerConstants;
import frc.robot.generated.TunerConstants;

public enum RobotIdentity {
    PEARRACUDA(new PearracudaTunerConstants()), // Competition Bot (5414)
    EVE(new Eve2TunerConstants()); // Practice Bot (9994)

    public final TunerConstants tunerConstants;

    private RobotIdentity(TunerConstants driveConstants) {
        this.tunerConstants = driveConstants;
    }

    public static RobotIdentity getRobotIdentity() {
        String rioSerial = RobotController.getSerialNumber();

        if (rioSerial.equals("032B4B61")) {
            return EVE;
        }

        return PEARRACUDA;
    }

    public static String getRobotIdentityString() {
        String rioSerial = RobotController.getSerialNumber();

        if (rioSerial.equals("032B4B61")) {
            return "EVE";
        } else if (rioSerial.equals("032B4B64")) {
            return "PEARRACUDA";
        } else {
            return "UNKNOWN";
        }
    }

    public static String getRoboRioSerial() {
        return RobotController.getSerialNumber();
    }
}
