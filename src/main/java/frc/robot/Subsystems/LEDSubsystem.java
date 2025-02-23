package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED ledStrip1;
    private final AddressableLED ledStrip2;
    private final AddressableLEDBuffer ledBuffer1;
    private final AddressableLEDBuffer ledBuffer2;
    private final Limelight limelight;
    private static final int LED_COUNT = 5;

    public LEDSubsystem(int pwmPort1, int pwmPort2, Limelight limelight) {
        this.ledStrip1 = new AddressableLED(pwmPort1);
        this.ledStrip2 = new AddressableLED(pwmPort2);
        this.ledBuffer1 = new AddressableLEDBuffer(LED_COUNT);
        this.ledBuffer2 = new AddressableLEDBuffer(LED_COUNT);
        this.limelight = limelight;

        ledStrip1.setLength(LED_COUNT);
        ledStrip2.setLength(LED_COUNT);

        ledStrip1.setData(ledBuffer1);
        ledStrip2.setData(ledBuffer2);
        
        ledStrip1.start();
        ledStrip2.start();
    }

    @Override
    public void periodic() {
        boolean targetVisible = limelight.targetInView;
        if (targetVisible) {
            setLEDsBlueAndYellow();
        } else {
            turnOffLEDs();
        }
    }

    private void setLEDsBlueAndYellow() {
        for (int i = 0; i < LED_COUNT; i++) {
            if (i % 2 == 0) {
                ledBuffer1.setRGB(i, 0, 0, 255); // Blue
                ledBuffer2.setRGB(i, 0, 0, 255); // Blue
            } else {
                ledBuffer1.setRGB(i, 255, 255, 0); // Yellow
                ledBuffer2.setRGB(i, 255, 255, 0); // Yellow
            }
        }
        ledStrip1.setData(ledBuffer1);
        ledStrip2.setData(ledBuffer2);
    }

    private void turnOffLEDs() {
        for (int i = 0; i < LED_COUNT; i++) {
            ledBuffer1.setRGB(i, 0, 0, 0);
            ledBuffer2.setRGB(i, 0, 0, 0);
        }
        ledStrip1.setData(ledBuffer1);
        ledStrip2.setData(ledBuffer2);
    }
}
