package frc.robot.subsystems.scoring.intake.pivot;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.csm.RunnableCommand;
import frc.robot.constants.Constants;

import java.util.Objects;

public abstract class IntakePivotSubsystem extends SubsystemBase {
    private static IntakePivotSubsystem instance;

    public static IntakePivotSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, switch (Constants.currentRobot) {
            case CADENZA -> DummyIntakePivotSubsystem::new; //PhysicalIntakePivotSubsystem::new;
            case SIM -> DummyIntakePivotSubsystem::new;
        });
    }

    protected abstract void setPivot(PivotState state);

    // Could be used in the future.
    protected abstract void stopPivot();

    // Could be used in the future
    public abstract boolean isAtTarget();

    public RunnableCommand pivotCommand(PivotState state) {
        return RunnableCommand.wrap(() -> setPivot(state));
    }
}
