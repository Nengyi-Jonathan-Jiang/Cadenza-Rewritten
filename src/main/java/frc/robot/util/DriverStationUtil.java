package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public class DriverStationUtil {
    public static boolean isRed() {
        return getAlliance() == DriverStation.Alliance.Red;
    }

    public static DriverStation.Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    }
}