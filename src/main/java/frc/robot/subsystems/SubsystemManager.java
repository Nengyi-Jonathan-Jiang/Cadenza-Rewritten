package frc.robot.subsystems;

import frc.lib.led.DummyLEDStrip;
import frc.lib.led.LEDStrip;
import frc.lib.led.PhysicalLEDStrip;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import static frc.robot.constants.Constants.currentRobot;

public class SubsystemManager {
    private static LEDStrip ledStrip;

    public static LEDStrip getLedStrip() {
        if (ledStrip == null) {
            ledStrip = switch (currentRobot) {
                case SIM -> new DummyLEDStrip();
                case CADENZA -> new PhysicalLEDStrip(0, 64);
            };
        }
        return ledStrip;
    }

    public static SwerveDriveSubsystem getSwerveDrive() {
        return TunerConstants.DriveTrain;
    }
}
