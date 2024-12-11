package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Objects;

public class AllianceChooser {
    private final SendableChooser<DriverStation.Alliance> chooser;

    public AllianceChooser() {
        chooser = new SendableChooser<>();
        switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
            case Red -> {
                chooser.setDefaultOption("Red", DriverStation.Alliance.Red);
                chooser.addOption("Blue", DriverStation.Alliance.Blue);
            }
            case Blue -> {
                chooser.addOption("Red", DriverStation.Alliance.Red);
                chooser.setDefaultOption("Blue", DriverStation.Alliance.Blue);
            }
        }
    }

    public void addSelfToSmartDashboardWithName(String name) {
        SmartDashboard.putData(name, chooser);
    }

    public DriverStation.Alliance getAlliance() {
        DriverStation.Alliance selectedAlliance = chooser.getSelected();
        // Assert that selected alliance is not null (should always pass)
        return Objects.requireNonNull(selectedAlliance);
    }

    public boolean isRedAlliance() {
        return getAlliance() == DriverStation.Alliance.Red;
    }

    public boolean isBlueAlliance() {
        return getAlliance() == DriverStation.Alliance.Blue;
    }
}
