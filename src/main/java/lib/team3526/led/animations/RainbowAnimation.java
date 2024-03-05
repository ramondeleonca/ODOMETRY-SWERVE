package lib.team3526.led.animations;

import edu.wpi.first.wpilibj.Timer;
import lib.team3526.led.framework.HyperAddressableLEDBuffer;
import lib.team3526.led.framework.HyperLEDAnimation;

public class RainbowAnimation extends HyperLEDAnimation {
    double duration;

    public RainbowAnimation(double duration) {
        this.duration = duration;
    }

    public void provider(HyperAddressableLEDBuffer data) {
        double timeFraction = (Timer.getFPGATimestamp() % duration) / duration;
        for (int i = 0; i < data.getLength(); i++) data.setHSV(i, (int)(timeFraction * 255), 255, 255);
    }
}
