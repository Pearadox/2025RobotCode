package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
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

    private static final Frequency DEFAULT_FREQUENCY = Percent.per(Second).of(25);

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
        setSolid(CORAL[0], CORAL[1], CORAL[2]);
    }

    public void periodic() {
        if (DriverStation.isDisabled()) {
            scrollingGradient(Color.kGreen, Color.kGold);
        } else {
            handleFlashing();
        }
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

    public void setSolid(Color c) {
        LEDPattern solidPattern = LEDPattern.solid(c);
        solidPattern.applyTo(ledBuffer);
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
        ;
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

    public void scrollingGradient(Color colorA, Color colorB) {
        LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colorA, colorB);
        LEDPattern pattern = base.scrollAtRelativeSpeed(DEFAULT_FREQUENCY);

        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }
}
