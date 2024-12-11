package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj2.command.*;
import frc.lib.led.LEDPattern;
import frc.lib.led.LEDStrip;
import frc.lib.oi.OI;
import frc.robot.constants.Controls;
import frc.robot.constants.DisplayInfo;
import frc.robot.constants.RobotInfo.HopperInfo;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.scoring.hopper.HopperSubsystem;
import frc.robot.subsystems.scoring.intake.IntakeSubsystem;
import frc.robot.subsystems.scoring.intake.pivot.PivotState;
import frc.robot.subsystems.scoring.shooter.ShooterSubsystem;
import frc.robot.subsystems.scoring.shooter.ShootingMode;

import java.util.Objects;

// TODO: rename this subsystem. The idea is that this subsystem takes
//  care of all note-related activities: shooting and intaking
public class ScoringSubsystem extends SubsystemBase {
    private ScoringSubsystem() {}

    private static ScoringSubsystem instance;

    public static ScoringSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance,
            ScoringSubsystem::new
        );
    }

    private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private final HopperSubsystem hopper = HopperSubsystem.getInstance();
    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    public Command intakeCommand() {
        return new SequentialCommandGroup(
            // Extend intake
            intake.pivotCommand(PivotState.EXTENDED),
            // Run intake until has note
            new ParallelCommandGroup(
                intake.pivotCommand(PivotState.EXTENDED),
                intake.intake(),
                hopper.runHopperCommand(HopperInfo.INTAKE_HOPPER_SPEED)
            ).until(hopper::hasNote),
            // Run intake backwards a bit so that the note can have more momentum going into the shooter
            new ParallelCommandGroup(
                intake.pivotCommand(PivotState.RETRACTED),
                hopper.runHopperCommand(HopperInfo.SLOW_BACKWARDS_HOPPER_SPEED),
                OI.getInstance().driverController().rumbleCommand(Controls.rumbleStrength)
            ).withTimeout(0.1),
            // Retract intake
            intake.pivotCommand(PivotState.RETRACTED)
        );
    }

    public Command outtakeCommand() {
        return new ParallelCommandGroup(
            intake.outtakeCommand(),
            hopper.runHopperCommand(-HopperInfo.INTAKE_HOPPER_SPEED)
        );
    }

    private Command feedAndSpinupCommand(double feedTime) {
        LEDStrip ledStrip = SubsystemManager.getLedStrip();

        return new InstantCommand(
            () -> ledStrip.usePattern(DisplayInfo.notReadyPattern)
        ).andThen(new ParallelCommandGroup(
            shooter.spinupCommand(),
            hopper.runHopperCommand(HopperInfo.INTAKE_HOPPER_SPEED)
                .withTimeout(feedTime)
                .andThen(new WaitUntilCommand(hopper::hasNote))
        ));
    }

    private Command shootCommand(ShootingMode shootingMode, double hopperSpeed, boolean requireSeparateShootButton, double feedTime) {
        LEDStrip ledStrip = SubsystemManager.getLedStrip();

        return feedAndSpinupCommand(feedTime).andThen(
            new ParallelCommandGroup(
                shooter.withShootingModeCommand(shootingMode),
                hopper.runHopperCommand(() -> {
                    if (requireSeparateShootButton)
                        if (!Controls.OperatorControls.FeedShooterButton.getAsBoolean())
                            return 0;

                    return hopperSpeed;

                }).onlyWhile(hopper::hasNote)
            ).andThen(() -> ledStrip.usePattern(LEDPattern.BLANK))
        );
    }

    public Command autoSpeakerCommand() {
        return shootCommand(
            ShootingMode.AUTO_SPEAKER,
            HopperInfo.INTAKE_HOPPER_SPEED,
            false,
            0
        );
    }

    public Command manualSpeakerCommand() {
        return shootCommand(
            ShootingMode.SPEAKER,
            HopperInfo.INTAKE_HOPPER_SPEED,
            false,
            0
        );
    }

    public Command launchCommand() {
        return shootCommand(
            ShootingMode.LAUNCH,
            HopperInfo.INTAKE_HOPPER_SPEED,
            true,
            0
        );
    }

    public Command ampCommand() {
        return shootCommand(
            ShootingMode.AMP,
            HopperInfo.AMP_HOPPER_SPEED,
            false,
            0.08
        );
    }
}
