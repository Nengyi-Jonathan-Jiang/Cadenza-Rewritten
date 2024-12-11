package frc.robot.subsystems.scoring.shooter.wheels;

import frc.csm.PackagePrivate;
import frc.robot.subsystems.scoring.shooter.ShootingMode;

@PackagePrivate
class DummyShooterWheelsSubsystem extends ShooterWheelsSubsystem {
    @Override
    public boolean isUpToSpeed() {
        return true;
    }

    @Override
    public void setShootingMode(ShootingMode shooting) {}
}
