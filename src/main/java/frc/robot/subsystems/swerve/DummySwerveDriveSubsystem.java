package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.csm.PackagePrivate;

@PackagePrivate
class DummySwerveDriveSubsystem extends SwerveDriveSubsystem {
    @Override
    public Rotation2d getOrientation() {
        return new Rotation2d();
    }

    @Override
    public void stop() {

    }

    @Override
    public void setControl(SwerveRequest request) {

    }

    @Override
    public void resetPoseTo(Pose2d pose) {

    }

    @Override
    public void resetOrientation() {

    }

    @Override
    public Pose2d getPose() {
        return null;
    }

    @Override
    public Command pathfindCommand(Pose2d targetPose) {
        return null;
    }

    @Override
    public Command driveFieldCentricCommand() {
        return null;
    }

    @Override
    public Command shootOnTheFlyCommand() {
        return null;
    }

    @Override
    public boolean isAligned() {
        return false;
    }

    @Override
    public boolean inShootingRange() {
        return false;
    }

    @Override
    public boolean inShootingSector() {
        return false;
    }

    @Override
    public boolean inSpinupRange() {
        return false;
    }
}
