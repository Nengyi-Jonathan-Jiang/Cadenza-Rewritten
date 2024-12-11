package frc.lib;

import edu.wpi.first.wpilibj.DriverStation;

public class DriverStationHelpers {
    public static DriverStation.Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    }
}
