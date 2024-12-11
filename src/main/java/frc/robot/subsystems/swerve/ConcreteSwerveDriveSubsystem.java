package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.Controls;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AimUtil;
import frc.robot.util.DriverStationUtil;

import static frc.robot.constants.RobotInfo.SwerveInfo.CURRENT_MAX_ROBOT_MPS;

public class ConcreteSwerveDriveSubsystem extends SwerveDriveSubsystem {
    private final Field2d field = new Field2d();
    private final CommandSwerveDrivetrain driveTrain;
    private final PhoenixPIDController headingPID;

    private final SlewRateLimiter limiterForward;
    private final SlewRateLimiter limiterSideways;

    public ConcreteSwerveDriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        this.driveTrain = new CommandSwerveDrivetrain(driveTrainConstants, modules);
        configurePathPlanner();
        headingPID = new PhoenixPIDController(8, 0, 0);
        headingPID.setTolerance(Rotation2d.fromDegrees(7.5).getRadians());

        limiterForward = new SlewRateLimiter(10000);
        limiterSideways = new SlewRateLimiter(10000);
    }

    public Command pathfindCommand(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
            6, 3,
            2 * Math.PI, 2 * Math.PI
        );
        return AutoBuilder.pathfindToPoseFlipped(
            targetPose,
            constraints
        );
    }

    public Command driveFieldCentricCommand() {
        return applyRequest(() -> {
            double forwardsSpeedRaw = Controls.DriverControls.getSwerveForwardAxis();
            double sidewaysSpeedRaw = Controls.DriverControls.getSwerveStrafeAxis();
            double rotationSpeedRaw = Controls.DriverControls.getSwerveRotationAxis();

            double rotationSpeedFactor = 1 + Math.hypot(forwardsSpeedRaw, sidewaysSpeedRaw);

            double forwardsSpeed = limiterForward.calculate(forwardsSpeedRaw * CURRENT_MAX_ROBOT_MPS);
            double sidewaysSpeed = limiterSideways.calculate(sidewaysSpeedRaw * CURRENT_MAX_ROBOT_MPS);
            double rotationSpeed = rotationSpeedRaw * rotationSpeedFactor * CURRENT_MAX_ROBOT_MPS;

            return new SwerveRequest.FieldCentric()
                .withVelocityX(forwardsSpeed)
                .withVelocityY(sidewaysSpeed)
                .withRotationalRate(rotationSpeed);
        });
    }

    public Command shootOnTheFlyCommand() {
        return applyRequest(() -> {
            SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle();
            request.HeadingController = headingPID;
            return request
                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagic)
                .withTargetDirection(
                    AimUtil.getSpeakerRotation(Controls.DriverControls.getSwerveStrafeAxis())
                )
                .withVelocityX(Controls.DriverControls.getSwerveForwardAxis() / 2)
                .withVelocityY(Controls.DriverControls.getSwerveStrafeAxis() / 2);
        });
    }

    @Override
    public void periodic() {
        driveTrain.periodic();

        field.setRobotPose(getPose());
        SmartDashboard.putData("field", field);
        SmartDashboard.putNumber("Drive/heading", getOrientation().getDegrees());
        SmartDashboard.putNumber("Drive/dist", AimUtil.getSpeakerDist());
        SmartDashboard.putBoolean("Drive/Can SOTF", Controls.canShootOnTheFly.getAsBoolean());
        SmartDashboard.putBoolean("Drive/SOTF", Controls.DriverControls.ShootOnTheFlyButton.getAsBoolean());
        SmartDashboard.putBoolean("Drive/Amp Align", Controls.DriverControls.AmpAlignButton.getAsBoolean());
        SmartDashboard.putBoolean("is red", DriverStationUtil.isRed());
    }

    @Override
    public void setControl(SwerveRequest request) {
        driveTrain.setControl(request);
    }

    @Override
    public void resetPoseTo(Pose2d pose) {
        driveTrain.seedFieldRelative(pose);
    }

    @Override
    public void simulationPeriodic() {
        driveTrain.simulationPeriodic();

        field.setRobotPose(getPose());
        SmartDashboard.putData("field", field);
    }

    public void resetOrientation() {
        driveTrain.resetOrientation();
    }

    @Override
    public Pose2d getPose() {
        return driveTrain.getPose();
    }

    @Override
    public Rotation2d getOrientation() {
        return driveTrain.getOrientation();
    }

    @Override
    public void stop() {
        setControl(new SwerveRequest.SwerveDriveBrake());
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
            () -> driveTrain.getState().Pose,
            driveTrain::seedFieldRelative,
            driveTrain::getChassisSpeeds,
            (speeds) -> setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                new PIDConstants(10, 0, 0),
                TunerConstants.kSpeedAt12VoltsMps,
                driveTrain.getDriveBaseRadius(),
                new ReplanningConfig()),
            DriverStationUtil::isRed,
            this);
    }

    public boolean inShootingRange() {
        return AimUtil.getSpeakerDist() <= 3.5;
    }

    public boolean inShootingSector() {
        Rotation2d rotation = AimUtil.getSpeakerRotation();
        return Math.abs(rotation.getDegrees()) <= 35;
    }

    public boolean isAligned() {
        if (!Robot.isInAuton()) {
            return headingPID.atSetpoint();
        } else {
            return Math.abs(AimUtil.getSpeakerOffset().getDegrees()) <= 5;
        }
    }

    public boolean inSpinupRange() {
        return AimUtil.getSpeakerDist() <= 4;
    }
}