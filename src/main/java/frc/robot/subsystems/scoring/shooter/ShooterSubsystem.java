package frc.robot.subsystems.scoring.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.csm.RunnableCommand;
import frc.robot.subsystems.scoring.shooter.pivot.ShooterPivotSubsystem;
import frc.robot.subsystems.scoring.shooter.wheels.ShooterWheelsSubsystem;

import java.util.Objects;

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem instance;

    public static ShooterSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance,
            ShooterSubsystem::new
        );
    }

    private final ShooterPivotSubsystem pivot = ShooterPivotSubsystem.getInstance();
    private final ShooterWheelsSubsystem wheels = ShooterWheelsSubsystem.getInstance();
    private ShootingMode shootingMode = ShootingMode.IDLE;

    public boolean isReady() {
        return pivot.isAtSetPoint() && wheels.isUpToSpeed();
    }

    public ShootingMode getShootingMode() {
        return this.shootingMode;
    }

    public void setShootingMode(ShootingMode shootingMode) {
        this.shootingMode = shootingMode;
        pivot.setShootingMode(shootingMode);
        wheels.setShootingMode(shootingMode);
    }

    public Command withShootingModeCommand(ShootingMode shootingMode) {
        return startEnd(
            () -> this.setShootingMode(shootingMode),
            () -> this.setShootingMode(ShootingMode.IDLE)
        );
    }

    public RunnableCommand setShootingModeCommand(ShootingMode shootingMode) {
        return RunnableCommand.wrap(() -> this.setShootingMode(shootingMode), this);
    }

    public Command spinupCommand() {
        return startEnd(
            () -> setShootingMode(ShootingMode.SPINUP),
            () -> setShootingMode(ShootingMode.IDLE)
        );
    }
}