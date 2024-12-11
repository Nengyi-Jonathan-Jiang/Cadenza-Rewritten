package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import frc.csm.PackagePrivate;
import frc.robot.constants.IDs;

import static frc.robot.constants.RobotInfo.ClimberInfo;

@PackagePrivate
class PhysicalClimberSubsystem extends ClimberSubsystem {
    private final CANSparkMax leftMotor, rightMotor;

    @PackagePrivate
    PhysicalClimberSubsystem() {
        leftMotor = new CANSparkMax(IDs.CLIMBER_LEFT, CANSparkMax.MotorType.kBrushless);
        rightMotor = new CANSparkMax(IDs.CLIMBER_RIGHT, CANSparkMax.MotorType.kBrushless);

        leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        rightMotor.setInverted(true);
    }

    public void setLeftSpeed(double speed) {
        leftMotor.set(speed * ClimberInfo.CLIMBER_SPEED);
    }

    public void setRightSpeed(double speed) {
        rightMotor.set(speed * ClimberInfo.CLIMBER_SPEED);
    }

    @Override
    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
}
