package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.generated.EveTunerConstants;
import frc.robot.generated.PearracudaTunerConstants;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public enum RobotIdentity {
    PEARRACUDA(new PearracudaTunerConstants()), // Competition Bot (5414)
    EVE(new EveTunerConstants()); // Practice Bot (9994)

    public final TunerConstants tunerConstants;

    private RobotIdentity(TunerConstants driveConstants) {
        this.tunerConstants = driveConstants;
    }

    @AutoLogOutput
    public static RobotIdentity getRobotIdentity() {
        String rioSerial = RobotController.getSerialNumber();

        Logger.recordOutput("RobotIdentity/RioSerial", rioSerial);

        if (rioSerial.equals("032B4B61")) {
            return EVE;
        }

        System.out.println("Running on an unrecognized RoboRIO! " + rioSerial);

        return PEARRACUDA;
    }
}
