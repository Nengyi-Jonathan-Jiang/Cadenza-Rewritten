package frc.robot.subsystems.scoring.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.scoring.intake.pivot.IntakePivotSubsystem;
import frc.robot.subsystems.scoring.intake.pivot.PivotState;
import frc.robot.subsystems.scoring.intake.rollers.IntakeRollersSubsystem;
import frc.robot.subsystems.scoring.intake.rollers.RollerSpeed;

import java.util.Objects;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;
    private final IntakePivotSubsystem pivot = IntakePivotSubsystem.getInstance();
    private final IntakeRollersSubsystem rollers = IntakeRollersSubsystem.getInstance();

    public static IntakeSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, IntakeSubsystem::new);
    }

    public Command pivotCommand(PivotState state) {
        return pivot.pivotCommand(state);
    }

    public Command outtakeCommand() {
        return pivot.pivotCommand(PivotState.RETRACTED)
            .alongWith(rollers.runRollersCommand(RollerSpeed.OUTTAKE));
    }

    public Command ampCommand() {
        return pivot.pivotCommand(PivotState.AMP)
            .alongWith(rollers.runRollersCommand(RollerSpeed.AMP));
    }

    public Command intake() {
        return pivot.pivotCommand(PivotState.EXTENDED)
            .alongWith(rollers.runRollersCommand(RollerSpeed.INTAKE));
    }
}
