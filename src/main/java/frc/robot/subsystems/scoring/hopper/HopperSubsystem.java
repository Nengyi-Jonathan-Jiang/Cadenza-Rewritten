package frc.robot.subsystems.scoring.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import java.util.Objects;
import java.util.function.DoubleSupplier;

public abstract class HopperSubsystem extends SubsystemBase {
    private static HopperSubsystem instance;
    private boolean irActive;

    public static HopperSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, switch (Constants.currentRobot) {
            case CADENZA -> PhysicalHopperSubsystem::new;
            case SIM -> DummyHopperSubsystem::new;
        });
    }

    public abstract void runHopper(double speed);

    public abstract void stopHopper();

    /**
     * Should return true if the hopper has a note in it
     */
    protected abstract boolean noteDetected();

    /**
     * Returns true if the IR sensor is disabled or the hopper detects a note
     */
    public boolean hasNote() {
        return !isIrEnabled() || noteDetected();
    }

    /**
     * Returns true if the IR sensor is disabled or the hopper does not detect a note
     */
    public boolean isClear() {
        return !isIrEnabled() || !noteDetected();
    }

    public final Command runHopperCommand(double speed) {
        return runHopperCommand(() -> speed);
    }

    public final Command runHopperCommand(DoubleSupplier speed) {
        return startEnd(() -> runHopper(speed.getAsDouble()), this::stopHopper);
    }

    // TODO: refactor to remove this method. This logic should be elsewhere
    public final Command runHopperIfNoNoteCommand(double speed) {
        return new Command() {
            @Override
            public void execute() {
                if (hasNote()) {
                    runHopper(speed);
                } else {
                    stopHopper();
                }
            }

            @Override
            public void end(boolean interrupted) {
                stopHopper();
            }
        };
    }

    public final Command toggleIrEnabled() {
        return new RunCommand(() -> irActive = !irActive);
    }

    public final boolean isIrEnabled() {
        return irActive;
    }
}
