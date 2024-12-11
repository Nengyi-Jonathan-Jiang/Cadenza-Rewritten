package frc.robot.subsystems.scoring.intake.pivot;

import frc.csm.PackagePrivate;

@PackagePrivate
class DummyIntakePivotSubsystem extends IntakePivotSubsystem {

    @Override
    protected void setPivot(PivotState state) {}

    @Override
    protected void stopPivot() {}

    @Override
    public boolean isAtTarget() {
        // Pretend that we are always at the target.
        return true;
    }
}
