package frc.robot.subsystems.scoring.shooter.pivot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.csm.PackagePrivate;
import frc.robot.constants.IDs;
import frc.robot.subsystems.scoring.shooter.ShootingMode;
import frc.robot.util.AimUtil;

import static frc.robot.constants.RobotInfo.ShooterInfo;

@PackagePrivate
class PhysicalShooterPivotSubsystem extends ShooterPivotSubsystem {
    private final CANSparkMax aimMotor;

    private final DutyCycleEncoder encoder;
    private final PIDController pid;

    @PackagePrivate
    PhysicalShooterPivotSubsystem() {
        CANSparkMax aimMotorLeft, aimMotorRight;

        aimMotorLeft = new CANSparkMax(IDs.SHOOTER_PIVOT_MOTOR_LEFT, CANSparkMax.MotorType.kBrushless);
        aimMotorLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);

        aimMotorRight = new CANSparkMax(IDs.SHOOTER_PIVOT_MOTOR_RIGHT, CANSparkMax.MotorType.kBrushless);
        aimMotorRight.setIdleMode(CANSparkBase.IdleMode.kBrake);

        aimMotor = aimMotorLeft;
        aimMotorRight.follow(aimMotorLeft, true);

        encoder = new DutyCycleEncoder(IDs.SHOOTER_PIVOT_ENCODER_DIO_PORT);

        pid = ShooterInfo.SHOOTER_AIM_PID_CONSTANTS.create();
        pid.setSetpoint(ShooterInfo.SHOOTER_IDLE_SETPOINT.angle());
    }

    public boolean isAtSetPoint() {
        double error = Math.abs(pid.getSetpoint() - encoder.getAbsolutePosition());
        return error < ShooterInfo.SHOOTER_PIVOT_ERROR;
    }

    @Override
    public void periodic() {
        double currentAngle = getCurrentAngle();
        double pidOutput = pid.calculate(currentAngle);

//        aimMotor.set(pidOutput);

        SmartDashboard.putNumber("Target Shooter Angle", pid.getSetpoint());
        SmartDashboard.putNumber("Current Shooter Angle", currentAngle);
        SmartDashboard.putNumber("Aim PID Output", pidOutput);
    }

    @Override
    public void setShootingMode(ShootingMode shootingMode) {
        double targetAngle = (switch (shootingMode) {
            case AUTO_SPEAKER, SPINUP -> AimUtil.getShooterSetpoint();
            case SPEAKER -> ShooterInfo.SHOOTER_AMP_SETPOINT;
            case AMP -> ShooterInfo.SHOOTER_AMP_SETPOINT;
            case IDLE -> ShooterInfo.SHOOTER_IDLE_SETPOINT;
            case LAUNCH -> ShooterInfo.SHOOTER_LAUNCH_SETPOINT;
        }).angle() + ShooterInfo.AngleOffset;

        pid.setSetpoint(targetAngle);
    }

    private double getCurrentAngle() {
        return encoder.getAbsolutePosition();
    }

    public void stop() {
        aimMotor.stopMotor();
    }
}
