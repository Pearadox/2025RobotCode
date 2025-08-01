package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.generated.EveTunerConstants;
import frc.robot.generated.PearracudaTunerConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.WallETunerConstants;

public enum RobotIdentity {
    PEARRACUDA(new PearracudaTunerConstants()), // Competition Bot (5414)
    EVE(new EveTunerConstants()), // Practice Bot (9994)
    WALLE(new WallETunerConstants()); // Programming Bot

    public final TunerConstants tunerConstants;

    private RobotIdentity(TunerConstants driveConstants) {
        this.tunerConstants = driveConstants;
    }

    public static RobotIdentity getRobotIdentity() {
        String rioSerial = RobotController.getSerialNumber();

        if (rioSerial.equals("032B4B61")) {
            return EVE;
        } else if (rioSerial.equals("032B4B64")) {
            return PEARRACUDA;
        }

        return WALLE; // TODO: find Wall-E serial (low priority)
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
