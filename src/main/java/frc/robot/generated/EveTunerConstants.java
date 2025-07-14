package frc.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class EveTunerConstants extends TunerConstants {
    private static final Slot0Configs STEER_GAINS = new Slot0Configs()
            .withKP(50)
            .withKI(0)
            .withKD(0.24539)
            .withKS(0.19817)
            .withKV(2.4066)
            .withKA(0.040542)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
            .withKP(0.62183) // 0.62183
            .withKI(0)
            .withKD(0)
            .withKS(0.10765)
            .withKV(0.82491)
            .withKA(0.0040132);

    private static final CANBus CAN_BUS = new CANBus("rio", "./logs/example.hoot");

    private static final double COUPLE_RATIO = 3.5714285714285716;
    private static final double DRIVE_RATIO = 6.746031746031747;
    private static final double TURN_RATIO = 21.428571428571427;
    
    private static final Distance WHEEL_RADIUS = Inches.of(2);

    private static final boolean LEFT_INVERTED = false;
    private static final boolean RIGHT_INVERTED = true;

    private static final int PIGEON_ID = 15;

    private static final Angle FL_ENCODER_OFFSET = Rotations.of(-0.30517578125);
    private static final Angle FR_ENCODER_OFFSET = Rotations.of(-0.04052734375);
    private static final Angle BL_ENCODER_OFFSET = Rotations.of(-0.23486328125);
    private static final Angle BR_ENCODER_OFFSET = Rotations.of(0.353271484375);

    private static final Distance FL_X_POS = Inches.of(12.375);
    private static final Distance FL_Y_POS = Inches.of(11.375);

    public EveTunerConstants() {
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
