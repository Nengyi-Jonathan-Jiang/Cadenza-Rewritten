package frc.robot.subsystems.scoring.shooter.wheels;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.scoring.shooter.ShootingMode;

import java.util.Objects;

public abstract class ShooterWheelsSubsystem extends SubsystemBase {
    private static ShooterWheelsSubsystem instance;

    public static ShooterWheelsSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance,
            PhysicalShooterWheelsSubsystem::new
        );
    }

    public abstract boolean isUpToSpeed();

    public abstract void setShootingMode(ShootingMode shooting);
}