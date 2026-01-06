package frc.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Eve2TunerConstants extends TunerConstants {
    private static final Slot0Configs STEER_GAINS = new Slot0Configs()
            .withKP(55 * 0.01) // try setting back to 55
            .withKI(0)
            .withKD(0) // try setting back to 0.25
            .withKS(0.17208) // try setting to zero
            .withKV(2.07937) // try setting to zero, this is likely the problem
            .withKA(0.040542) // try setting to zero
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
            .withKP(0.62183)
            .withKI(0)
            .withKD(0)
            .withKS(0.08846)
            .withKV(0.67)
            .withKA(0.0040132);

    private static final CANBus CAN_BUS = new CANBus("Drivetrain", "./logs/example.hoot");

    private static final double COUPLE_RATIO = (54. / 14.); // ~ 3.86:1 (note: measure and verify this)
    private static final double DRIVE_RATIO = (54. / 14.) * (25. / 32.) * (30. / 15.); // ~ 6.03:1
    private static final double TURN_RATIO = 287. / 11.; // ~ 26.09:1

    private static final Distance WHEEL_RADIUS = Inches.of(2); // todo: characterize

    private static final boolean LEFT_INVERTED = false;
    private static final boolean RIGHT_INVERTED = true;

    private static final int PIGEON_ID = 15;

    private static final Angle FL_ENCODER_OFFSET = Rotations.of(-0.1591796875);
    private static final Angle FR_ENCODER_OFFSET = Rotations.of(0.246826171875);
    private static final Angle BL_ENCODER_OFFSET = Rotations.of(-0.3603515625);
    private static final Angle BR_ENCODER_OFFSET = Rotations.of(-0.113525390625);

    private static final Distance FL_X_POS = Inches.of(11.375); // 28/2 - 2.625
    private static final Distance FL_Y_POS = Inches.of(11.375); // 28/2 - 2.625

    public Eve2TunerConstants() {
        super(
                STEER_GAINS,
                DRIVE_GAINS,
                CAN_BUS,
                COUPLE_RATIO,
                DRIVE_RATIO,
                TURN_RATIO,
                WHEEL_RADIUS,
                LEFT_INVERTED,
                RIGHT_INVERTED,
                PIGEON_ID,
                FL_ENCODER_OFFSET,
                FR_ENCODER_OFFSET,
                BL_ENCODER_OFFSET,
                BR_ENCODER_OFFSET,
                FL_X_POS,
                FL_Y_POS);
    }
}
