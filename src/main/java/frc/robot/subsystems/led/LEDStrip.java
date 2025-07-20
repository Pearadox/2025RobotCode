package frc.robot.subsystems.led;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.RobotContainer;
import frc.robot.util.SmarterDashboard;
import java.util.function.BooleanSupplier;

public class LEDStrip extends SubsystemBase {
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    // private AddressableLEDBufferView DynamoBufferViewLeft;
    // private AddressableLEDBufferView DynamoBufferViewRight;

    private static final LEDStrip LEDSTRIP = new LEDStrip();

    public static LEDStrip getInstance() {
        return LEDSTRIP;
    }

    private static final Color kRavenBlack = new Color("#101820");
    private static final Color kDynamoOrange = new Color("#FF2700");
    private static final Color kSpaceCityBlue = new Color("#92C3F1");
    private static final Color kAstrosLightOrange = new Color("#F4871E");
    private static final Color kAstrosOrange = new Color("#EB6E1F");
    private static final Color kAstrosNavy = new Color("#002D62");

    public LEDStrip() {
        led = new AddressableLED(LEDConstants.PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.NUM_LEDS);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    public void periodic() {
        if (DriverStation.isDisabled()) {
            // setGradient(LEDConstants.SCROLL_FREQ, kDynamoOrange, kAstrosLightOrange, kAstrosNavy,
            // kAstrosLightOrange);
            setGradient(
                    LEDConstants.SCROLL_FREQ,
                    gammaCorrect(kAstrosOrange, 2.2),
                    gammaCorrect(kAstrosLightOrange, 2.2),
                    gammaCorrect(kAstrosNavy, 1));
        } else {
            // setGradient(Percent.per(Second).of(getRate()), kDynamoOrange, kDynamoOrange, kSpaceCityBlue);
            setBreathing(kDynamoOrange);
        }
        led.setData(ledBuffer);

        SmarterDashboard.putNumber("LED/Rate", getRate());
    }

    public double getRate() {
        return Math.sqrt(Math.pow(RobotContainer.driverController.getLeftX(), 2)
                + Math.pow(RobotContainer.driverController.getLeftY(), 2));
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
        return new RunCommand(() -> setBlinking(LEDConstants.BLINK_PERIOD, Color.kGreen), this)
                .withTimeout(LEDConstants.BLINKING_DURATION);
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

    private void setGradient(Frequency frequency, Color... colors) {
        LEDPattern gradient =
                LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors).scrollAtRelativeSpeed(frequency);
        gradient.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    private void setBlinking(Time period, Color color) {
        LEDPattern blinkPattern =
                LEDPattern.solid(color).blink(period.div(2.0)).atBrightness(LEDConstants.BLINK_BRIGHTNESS);
        blinkPattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    private void setBreathing(Color color) {
        LEDPattern breathePattern = LEDPattern.solid(color).breathe(LEDConstants.BREATHE_PERIOD);
        breathePattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    private Color gammaCorrect(Color color, double gamma) {
        return new Color(gamma(color.red, gamma), gamma(color.green, gamma), gamma(color.blue, gamma));
    }

    private int gamma(double val, double gamma) {
        return Math.min(255, (int) Math.round(255 * Math.pow(val, gamma)));
    }
}
