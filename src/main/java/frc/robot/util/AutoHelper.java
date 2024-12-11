package frc.robot.util;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.led.LEDStrip;
import frc.robot.Robot;
import frc.robot.constants.Controls;
import frc.robot.constants.RobotInfo.SwerveInfo;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.hopper.HopperSubsystem;
import frc.robot.subsystems.scoring.intake.IntakeSubsystem;
import frc.robot.subsystems.scoring.intake.pivot.PivotState;
import frc.robot.subsystems.scoring.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class AutoHelper {
    private static final SwerveDriveSubsystem swerve = SubsystemManager.getSwerveDrive();
    private static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static final HopperSubsystem hopper = HopperSubsystem.getInstance();
    private static final LEDStrip ledStrip = SubsystemManager.getLedStrip();
    private static final ScoringSubsystem scoring = ScoringSubsystem.getInstance();

    public static Command follow(String pathName) {
        return swerve.followChoreoPath(pathName, true);
    }

    public static Command intakeWhileMoving(String pathName) {
        ChoreoTrajectory trajectory = Choreo.getTrajectory(pathName);
        double time = trajectory.getTotalTime();
        double startTime = time - SwerveInfo.TIME_BEFORE_INTAKE_START;
        return Commands.parallel(
            follow(pathName),
            startTime < 0 ?
                scoring.intakeCommand() :
                Commands.sequence(
                    new WaitCommand(startTime),
                    scoring.intakeCommand()
                )
        );
    }

    public static Command shootOnTheFlyCommand(String pathName) {
        /*
         * max velocity of trajectory during SOTF portion must not be too high (<=2 m/s),
         * should generate path so that bot is facing speaker during SOTF portion
         */
        ChoreoTrajectory trajectory = Choreo.getTrajectory(pathName);
        Command shooterAutoSpinupCommand = shooter.spinupCommand().onlyWhile(Controls.spinupTrigger);
        Command shooterShootOnTheFlyCommand = scoring.autoSpeakerCommand();
        return Commands.deadline(
            AutoHelper.follow(pathName),
            Commands.sequence(
                Commands.deadline(
                    new WaitCommand(trajectory.getTotalTime() - SwerveInfo.TIME_BEFORE_INTAKE_START),
                    Commands.sequence(
                        Commands.waitUntil(Controls.spinupTrigger),
                        shooterAutoSpinupCommand.until(Controls.canShootOnTheFly),
                        shooterShootOnTheFlyCommand)
                ),
                scoring.intakeCommand()
            )
        );
    }

    public static Command followThenShoot(String pathname) {
        ChoreoTrajectory trajectory = Choreo.getTrajectory(pathname);
        return Commands.sequence(
            swerve.followChoreoPath(trajectory, true),
            shoot()
        );
    }

    public static Command shoot() {
        return Commands.parallel(
            Commands.race(
                new WaitCommand(Robot.isReal() ? 1.5 : 0),
                scoring.autoSpeakerCommand(),
                intake.pivotCommand(PivotState.EXTENDED)
            )
        );
    }


    public static Command ampPrepCommand() {
        return Commands.parallel(
                Commands.run(intake::outtakeCommand),
                Commands.run(() -> intake.pivotCommand(PivotState.EXTENDED))
            )
            .onlyWhile(hopper::hasNote);
    }
}
