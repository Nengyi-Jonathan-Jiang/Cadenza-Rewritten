package frc.robot.subsystems.scoring.shooter.pivot;

import frc.csm.PackagePrivate;
import frc.robot.subsystems.scoring.shooter.ShootingMode;

@PackagePrivate
class DummyShooterPivotSubsystem extends ShooterPivotSubsystem {
    @PackagePrivate
    DummyShooterPivotSubsystem() {}

    @Override
    public void stop() {}

    @Override
    public boolean isAtSetPoint() {return false;}

    @Override
    public void setShootingMode(ShootingMode shootingMode) {
    }
}
