package frc.robot.subsystems.scoring.intake.pivot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.csm.PackagePrivate;
import frc.robot.Robot;
import frc.robot.constants.IDs;
import frc.robot.constants.RobotInfo.IntakeInfo;

@PackagePrivate
class PhysicalIntakePivotSubsystem extends IntakePivotSubsystem {
    private final CANSparkMax pivotMotorLeft;
    private final CANSparkMax pivotMotorRight;
    private final DutyCycleEncoder encoder;

    private final ProfiledPIDController pivotPID;

    @PackagePrivate
    PhysicalIntakePivotSubsystem() {
        pivotMotorLeft = new CANSparkMax(IDs.INTAKE_PIVOT_MOTOR_LEFT,
            CANSparkMax.MotorType.kBrushless);
        pivotMotorRight = new CANSparkMax(IDs.INTAKE_PIVOT_MOTOR_RIGHT,
            CANSparkMax.MotorType.kBrushless);
        pivotMotorLeft.restoreFactoryDefaults();
        pivotMotorRight.restoreFactoryDefaults();

        pivotMotorLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivotMotorRight.setIdleMode(CANSparkBase.IdleMode.kBrake);

        pivotMotorLeft.setSmartCurrentLimit(40);
        pivotMotorRight.setSmartCurrentLimit(40);

        pivotPID = new ProfiledPIDController(
            IntakeInfo.INTAKE_PIVOT_PID_CONSTANTS.kp(),
            IntakeInfo.INTAKE_PIVOT_PID_CONSTANTS.ki(),
            IntakeInfo.INTAKE_PIVOT_PID_CONSTANTS.kd(),
            new TrapezoidProfile.Constraints(
                2, 2
            )
        );
        // TODO: tune this number?
        pivotPID.setTolerance(0.05);

        encoder = new DutyCycleEncoder(IDs.INTAKE_ENCODER_DIO_PORT);
        encoder.setDistancePerRotation(0.75);

        pivotMotorRight.setInverted(true);
        setPivot(PivotState.RETRACTED);
    }

    @Override
    protected void setPivot(PivotState state) {
        pivotPID.setGoal(switch (state) {
            case EXTENDED -> IntakeInfo.INTAKE_PIVOT_EXTENDED_SETPOINT;
            case RETRACTED -> IntakeInfo.INTAKE_PIVOT_DEFAULT_SETPOINT;
            case AMP -> IntakeInfo.INTAKE_PIVOT_AMP_SETPOINT;
        });
    }

    @Override
    protected void stopPivot() {
        pivotPID.setGoal(getCurrentAngle());
    }

    private double getCurrentAngle() {
        return ((encoder.get() + 0.15) % 1 + 1) % 1;
    }

    @Override
    public boolean isAtTarget() {
        return pivotPID.atGoal();
    }

    @Override
    public void periodic() {
        super.periodic();

        double pidOutput = -pivotPID.calculate(getCurrentAngle());
        SmartDashboard.putNumber("Intake Pivot Angle", getCurrentAngle());
        SmartDashboard.putNumber("Intake Pivot Goal", pivotPID.getGoal().position);
        SmartDashboard.putNumber("Intake Pivot Setpoint", pivotPID.getSetpoint().position);
        SmartDashboard.putNumber("Intake PID output", pidOutput);

        pivotMotorRight.set(pidOutput);
        pivotMotorLeft.set(pidOutput);

        CANSparkBase.IdleMode idleMode = Robot.getDisabled()
            ? CANSparkBase.IdleMode.kCoast
            : CANSparkBase.IdleMode.kBrake;

        if (pivotMotorLeft.getIdleMode() != idleMode) {
            pivotMotorLeft.setIdleMode(idleMode);
        }
        if (pivotMotorRight.getIdleMode() != idleMode) {
            pivotMotorRight.setIdleMode(idleMode);
        }
    }
}