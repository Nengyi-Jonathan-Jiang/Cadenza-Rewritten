package frc.robot.subsystems.scoring.intake.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import java.util.Objects;

public abstract class IntakeRollersSubsystem extends SubsystemBase {

    private static IntakeRollersSubsystem instance;

    public static IntakeRollersSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, switch (Constants.currentRobot) {
            case CADENZA -> PhysicalIntakeRollersSubsystem::new;
            case SIM -> DummyIntakeRollersSubsystem::new;
        });
    }

    protected abstract void setRollersSpeed(RollerSpeed speed);

    protected abstract void stopRollers();

    public Command runRollersCommand(RollerSpeed speed) {
        return startEnd(() -> setRollersSpeed(speed), this::stopRollers);
    }
}
