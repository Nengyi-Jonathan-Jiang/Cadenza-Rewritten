package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.network.LimelightHelpers;

public class CommandSwerveDrivetrain extends SwerveDrivetrain {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private final double driveBaseRadius;
    private double lastSimTime;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        seedFieldRelative(new Pose2d());
        if (Utils.isSimulation()) {
            startSimThread();
        }

        double driveBaseRadius = 0;
        for (var swerveModule : modules) {
            double moduleRadius = Math.hypot(swerveModule.LocationX, swerveModule.LocationY);
            driveBaseRadius = Math.max(driveBaseRadius, moduleRadius);
        }
        this.driveBaseRadius = driveBaseRadius;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public double getDriveBaseRadius() {
        return driveBaseRadius;
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        /* use the measured time delta, get battery voltage from WPILib */
        try (Notifier simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        })) {
            simNotifier.startPeriodic(kSimLoopPeriod);
        }
    }

    public void periodic() {
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if (isPoseEstimateValid(poseEstimate)) {
            addVisionMeasurement(poseEstimate.pose, Timer.getFPGATimestamp());
        }
    }

    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(0.020, RobotController.getBatteryVoltage());
    }

    public void resetOrientation() {
        m_pigeon2.setYaw(0);
    }

    private boolean isPoseEstimateValid(LimelightHelpers.PoseEstimate poseEstimate) {
        if (poseEstimate == null) return false;
        Pose2d pose2d = poseEstimate.pose;
        Translation2d trans = pose2d.getTranslation();
        if (trans.getX() == 0 && trans.getY() == 0) {
            return false;
        }
        return !Robot.isInAuton();
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public Rotation2d getOrientation() {
        return m_pigeon2.getRotation2d();
    }
}
