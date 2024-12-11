package frc.robot.subsystems.scoring.intake.rollers;

import com.revrobotics.CANSparkMax;
import frc.csm.PackagePrivate;
import frc.robot.constants.IDs;
import frc.robot.constants.RobotInfo.IntakeInfo;

@PackagePrivate
class PhysicalIntakeRollersSubsystem extends IntakeRollersSubsystem {
    private final CANSparkMax rollersMotor;

    @PackagePrivate
    PhysicalIntakeRollersSubsystem() {
        rollersMotor = new CANSparkMax(IDs.INTAKE_MOTOR,
            CANSparkMax.MotorType.kBrushless);
    }

    protected void setRollersSpeed(double speed) {
        rollersMotor.set(speed);
    }

    @Override
    protected void setRollersSpeed(RollerSpeed speed) {
        setRollersSpeed(switch (speed) {
            case INTAKE -> IntakeInfo.INTAKE_SPEED;
            case OUTTAKE, AMP -> -IntakeInfo.INTAKE_SPEED;
        });
    }

    @Override
    protected void stopRollers() {
        rollersMotor.set(0);
    }
}