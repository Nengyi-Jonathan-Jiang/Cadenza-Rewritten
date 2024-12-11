package frc.robot.subsystems.scoring.hopper;

import frc.csm.PackagePrivate;

@PackagePrivate
class DummyHopperSubsystem extends HopperSubsystem {

    @Override
    public void runHopper(double speed) {

    }

    @Override
    public void stopHopper() {

    }

    @Override
    protected boolean noteDetected() {
        return true;
    }
}
