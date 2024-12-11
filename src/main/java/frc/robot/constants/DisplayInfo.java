package frc.robot.constants;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.led.LEDPattern;
import frc.lib.led.PhasingLEDPattern;
import frc.lib.led.SolidLEDPattern;


public final class DisplayInfo {
    public static final Color8Bit readyColor = new Color8Bit(0, 255, 0);
    public static final LEDPattern readyPattern = new PhasingLEDPattern(readyColor, 2);
    public static final Color8Bit notReadyColor = new Color8Bit(255, 0, 0);
    public static final LEDPattern notReadyPattern = new SolidLEDPattern(notReadyColor);

    public static final LEDPattern intakePattern = (ledIdx, time) -> new Color8Bit(
        Color.fromHSV(
            140 + (int) (8 * Math.sin(time * 5 + ledIdx * 0.5)),
            255,
            255
        )
    );
}
