package frc.lib.led;

import edu.wpi.first.wpilibj.util.Color8Bit;

public interface LEDPattern {
    LEDPattern BLANK = (led, time) -> new Color8Bit(0, 0, 0);

    Color8Bit get(int led, double time);
}
