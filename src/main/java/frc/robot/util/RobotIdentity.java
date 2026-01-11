package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;

public enum RobotIdentity {
    PEARRACUDA, // Competition Bot (5414)
    EVE; // Practice Bot (9994)

    private RobotIdentity() {}

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
