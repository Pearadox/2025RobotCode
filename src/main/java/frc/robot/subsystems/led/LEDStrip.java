package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import java.util.function.BooleanSupplier;

public class LEDStrip extends SubsystemBase {
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private static final LEDStrip LEDSTRIP = new LEDStrip();

    public static LEDStrip getInstance() {
        return LEDSTRIP;
    }

    public LEDStrip() {
        led = new AddressableLED(LEDConstants.PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.NUM_LEDS);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    public void periodic() {
        if (DriverStation.isDisabled()) {
            setGradient(Color.kGreen, Color.kGold);
        }
        led.setData(ledBuffer);
    }

    public Command defaultCommand(BooleanSupplier isCoralSupplier) {
        return new RunCommand(() -> setCoralOrAlgae(isCoralSupplier), this);
    }

    public Command aligning(BooleanSupplier isAligned) {
        return new RunCommand(() -> setBreathing(Color.kViolet), this)
                .until(isAligned)
                .andThen(success());
    }

    public Command success() {
        return new RunCommand(() -> setBlinking(Color.kGreen), this).withTimeout(LEDConstants.BLINKING_DURATION);
    }

    private void setCoralOrAlgae(BooleanSupplier isCoralSupplier) {
        setSolid(isCoralSupplier.getAsBoolean() ? Color.kCoral : Color.kMediumAquamarine);
    }

    private void setSolid(Color color) {
        LEDPattern solidPattern = LEDPattern.solid(color);
        solidPattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    private void setGradient(Color... colors) {
        LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors)
                .scrollAtRelativeSpeed(LEDConstants.SCROLL_FREQ);
        gradient.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    private void setBlinking(Color color) {
        LEDPattern blinkPattern = LEDPattern.solid(color)
                .blink(LEDConstants.BLINK_PERIOD.div(2.0))
                .atBrightness(LEDConstants.BLINK_BRIGHTNESS);
        blinkPattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    private void setBreathing(Color color) {
        LEDPattern breathePattern = LEDPattern.solid(color).breathe(LEDConstants.BREATHE_PERIOD);
        breathePattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }
}
