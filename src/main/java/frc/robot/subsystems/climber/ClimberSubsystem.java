package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import java.util.Objects;
import java.util.function.DoubleSupplier;

public abstract class ClimberSubsystem extends SubsystemBase {
    private static ClimberSubsystem instance;

    public static ClimberSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, switch (Constants.currentRobot) {
            case CADENZA -> PhysicalClimberSubsystem::new;
            case SIM -> DummyClimberSubsystem::new;
        });
    }

    public abstract void setLeftSpeed(double speed);

    public abstract void setRightSpeed(double speed);

    public final Command runWithSpeedsCommand(double leftSpeed, double rightSpeed) {
        return runWithSpeedsCommand(() -> leftSpeed, () -> rightSpeed);
    }

    public final Command runWithSpeedsCommand(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
        return startEnd(() -> {
            setLeftSpeed(leftSpeed.getAsDouble());
            setRightSpeed(rightSpeed.getAsDouble());
        }, this::stop);
    }

    public abstract void stop();
}
