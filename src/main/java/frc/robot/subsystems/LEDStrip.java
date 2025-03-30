package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SmarterDashboard;

public class LEDStrip extends SubsystemBase {
    private AddressableLED led;
    AddressableLEDBuffer ledBuffer;
    private final int numLeds;
    private boolean isFlashing = false;
    private boolean hasGamePiece = false;
    private double lastFlashTime;
    private boolean flashOn = false;
    private boolean isCoral = true;

    private static final int[] ALGAE = {0, 0, 128};
    private static final int[] CORAL = {200, 200, 200};

    private enum LEDState {
        kSolid,
        kFlashing
    }

    private static final LEDStrip LEDSTRIP = new LEDStrip(28, 9);

    public static LEDStrip getInstance() {
        return LEDSTRIP;
    }

    private LEDState ledMode = LEDState.kSolid;

    public LEDStrip(int numOfLeds, int port) {
        led = new AddressableLED(port);
        numLeds = numOfLeds;
        ledBuffer = new AddressableLEDBuffer(numLeds);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
        // setSolid(CORAL[0], CORAL[1], CORAL[2]);
        // default config
        LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kGreen, Color.kYellow);
        LEDPattern pattern = base.breathe(Time.ofBaseUnits(2, Seconds));
        LEDPattern scroll = base.scrollAtRelativeSpeed(Percent.per(Second).of(2));
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    public void periodic() {
        handleFlashing();
        SmarterDashboard.putString("LEDs/LED Mode", ledMode.toString());
    }

    public void setSolid(int r, int g, int b) {
        ledMode = LEDState.kSolid;
        isFlashing = false;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        led.setData(ledBuffer);
    }

    public void setLEDsCoralMode() {
        isCoral = true;
        if (!hasGamePiece) {
            setSolid(CORAL[0], CORAL[1], CORAL[2]);
        }
    }

    public void setLEDsAlgaeMode() {
        isCoral = false;
        if (!hasGamePiece) {
            setSolid(ALGAE[0], ALGAE[1], ALGAE[2]);
        }
    }

    public void setLEDsFlash() {
        ledMode = LEDState.kFlashing;
        isFlashing = true;
    }

    public void stopFlashing() {
        isFlashing = false;
        if (isCoral) {
            setSolid(CORAL[0], CORAL[1], CORAL[2]);
        } else if (!isCoral) {
            setSolid(ALGAE[0], ALGAE[1], ALGAE[2]);
        }
    }

    public void gamePieceIntook() {
        hasGamePiece = true;
        setLEDsFlash();
    }

    public void gamePieceScored() {
        hasGamePiece = false;
        stopFlashing();
    }

    public void handleFlashing() {
        if (isFlashing) {
            if (Timer.getFPGATimestamp() - lastFlashTime > 0.5) {
                flashOn = !flashOn;
                if (flashOn) {
                    if (isCoral) setSolid(CORAL[0], CORAL[1], CORAL[2]);
                    else if (!isCoral) setSolid(ALGAE[0], ALGAE[1], ALGAE[2]);
                }
            } else setSolid(0, 0, 0);
            lastFlashTime = Timer.getFPGATimestamp();
        }
    }
}
