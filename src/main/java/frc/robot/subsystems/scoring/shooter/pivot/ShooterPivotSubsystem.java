package frc.robot.subsystems.scoring.shooter.pivot;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.scoring.shooter.ShootingMode;

import java.util.Objects;

public abstract class ShooterPivotSubsystem extends SubsystemBase {
    private static ShooterPivotSubsystem instance;

    public static ShooterPivotSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, switch (Constants.currentRobot) {
            case CADENZA -> PhysicalShooterPivotSubsystem::new;
            case SIM -> DummyShooterPivotSubsystem::new;
        });
    }

    public abstract void stop();

    public abstract boolean isAtSetPoint();

    public abstract void setShootingMode(ShootingMode shootingMode);
}
