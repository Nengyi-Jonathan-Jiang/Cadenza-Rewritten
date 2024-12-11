package frc.robot.subsystems.swerve;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.DriverStationUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import static frc.robot.constants.RobotInfo.SwerveInfo.*;

public abstract class SwerveDriveSubsystem extends SubsystemBase {

    public abstract Rotation2d getOrientation();

    public abstract void stop();

    protected Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> setControl(requestSupplier.get()));
    }

    public abstract void setControl(SwerveRequest request);

    public abstract void resetPoseTo(Pose2d pose);

    public abstract void resetOrientation();

    public abstract Pose2d getPose();

    public abstract Command pathfindCommand(Pose2d targetPose);

    public abstract Command driveFieldCentricCommand();

    public abstract Command shootOnTheFlyCommand();

    /**
     * Returns a command that makes the robot follow a Choreo path using the ChoreoLib library.
     *
     * @param trajectory    The Choreo trajectory to follow.
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path
     */
    public Command followChoreoPath(ChoreoTrajectory trajectory, boolean resetPosition) {
        List<Command> commands = new ArrayList<>();

        if (resetPosition) {
            commands.add(runOnce(() -> resetPoseTo(
                switch (DriverStationUtil.getAlliance()) {
                    case Red -> trajectory.getFlippedInitialPose();
                    case Blue -> trajectory.getInitialPose();
                }
            )));
        }

        commands.add(choreoSwerveCommand(trajectory));
        return Commands.sequence(commands.toArray(Command[]::new));
    }


    /**
     * Returns a command that makes the robot follow a Choreo path using the ChoreoLib library.
     *
     * @param pathName      The name of a path located in the "deploy/choreo" directory
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path
     */
    public Command followChoreoPath(String pathName, boolean resetPosition) {
        return followChoreoPath(Choreo.getTrajectory(pathName), resetPosition);
    }

    public abstract boolean isAligned();

    private Command choreoSwerveCommand(ChoreoTrajectory trajectory) {
        return Choreo.choreoSwerveCommand(
            trajectory,
            this::getPose,
            choreoX,
            choreoY,
            choreoRotation,
            (ChassisSpeeds speeds) -> setControl(
                new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)
            ),
            DriverStationUtil::isRed,
            this
        );
    }

    public abstract boolean inShootingRange();

    public abstract boolean inShootingSector();

    public abstract boolean inSpinupRange();
}