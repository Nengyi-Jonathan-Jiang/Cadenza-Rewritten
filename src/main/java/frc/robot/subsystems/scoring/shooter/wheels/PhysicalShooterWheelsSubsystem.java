package frc.robot.subsystems.scoring.shooter.wheels;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.csm.PackagePrivate;
import frc.robot.constants.IDs;
import frc.robot.constants.InterpolatingTables;
import frc.robot.subsystems.scoring.shooter.ShootingMode;
import frc.robot.tuning.TunableNumberSource;
import frc.robot.util.AimUtil;

import static frc.robot.constants.RobotInfo.ShooterInfo.*;

@PackagePrivate
class PhysicalShooterWheelsSubsystem extends ShooterWheelsSubsystem {
    private final CANSparkFlex shooterMotor;
    private final PIDController shooterPID;

    // TODO: remove these and put somewhere else
    private static final double default_kp = 0;
    private static final double default_ki = 0;
    private static final double default_kd = 0;
    private static final double default_speed_multiplier = 1;
    private final TunableNumberSource shooter_speed_multiplier;
    private final TunableNumberSource kp;
    private final TunableNumberSource ki;
    private final TunableNumberSource kd;

    private ShootingMode shootingMode;

    @PackagePrivate
    PhysicalShooterWheelsSubsystem() {
        shooterMotor = new CANSparkFlex(IDs.SHOOTER_SHOOTER_LEFT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);


        shootingMode = ShootingMode.IDLE;

        kp = new TunableNumberSource("shooter kp", default_kp);
        ki = new TunableNumberSource("shooter ki", default_ki);
        kd = new TunableNumberSource("shooter kd", default_kd);
        shooterPID = new PIDController(kp.valueAsDouble(), ki.valueAsDouble(), kd.valueAsDouble());
        kp.addListener(shooterPID::setP);
        ki.addListener(shooterPID::setI);
        kd.addListener(shooterPID::setD);

        shooter_speed_multiplier = new TunableNumberSource(
            "shooter multiplier",
            default_speed_multiplier
        );
    }

    private double getCurrentSpeed() {
        return shooterMotor.getEncoder().getVelocity();
    }

    private double getTargetSpeed() {
        return switch (shootingMode) {
            case AUTO_SPEAKER, SPINUP -> AimUtil.getShooterSetpoint().speed() / 10;
            case SPEAKER -> SHOOTER_SPEAKER_SETPOINT.speed() / 10;
            case AMP -> SHOOTER_AMP_SETPOINT.speed() / 10;
            case IDLE -> SHOOTER_IDLE_SETPOINT.speed() / 10;
            case LAUNCH -> SHOOTER_LAUNCH_SETPOINT.speed() / 10;
        } * shooter_speed_multiplier.valueAsDouble();
    }

    private double applyPIDControl(double targetSpeed) {
        double currentSpeed = getCurrentSpeed();
        double currentSpeedPercent = currentSpeed / targetSpeed;
        double controllerOutput = shooterPID.calculate(currentSpeedPercent, 1) * targetSpeed;

        if (shooterMotor.getMotorTemperature() >= 80.0) {
            shooterMotor.stopMotor();
        } else {
            shooterMotor.setVoltage(controllerOutput);
        }

        return controllerOutput;
    }

    @Override
    public void periodic() {
        InterpolatingTables.update();
        kp.update();
        ki.update();
        kd.update();

        double c = applyPIDControl(getTargetSpeed());

        SmartDashboard.putNumber("target shooter speed", getTargetSpeed());
        SmartDashboard.putNumber("current shooter speed", getCurrentSpeed());
        SmartDashboard.putNumber("shooter pid output", c);
        SmartDashboard.putString("shooting mode", shootingMode.toString());

        SmartDashboard.putString(
            "shooter pid values",
            shooterPID.getP() + " " + shooterPID.getI() + " " + shooterPID.getD()
        );
    }

    public void setShootingMode(ShootingMode shootingMode) {
        if (this.shootingMode != shootingMode) {
            shooterPID.reset();
        }
        this.shootingMode = shootingMode;
    }

    @Override
    public boolean isUpToSpeed() {
        double tolerance = shootingMode == ShootingMode.AMP ? 0.02 : 0.05;

        return Math.abs(getCurrentSpeed()) >= Math.abs(getTargetSpeed()) * (1 - tolerance);
    }
}