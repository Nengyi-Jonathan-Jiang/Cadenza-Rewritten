// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.led.LEDStrip;
import frc.lib.led.PhasingLEDPattern;
import frc.lib.led.SolidLEDPattern;
import frc.lib.oi.OI;
import frc.robot.constants.Controls;
import frc.robot.constants.Controls.DriverControls;
import frc.robot.constants.InterpolatingTables;
import frc.robot.constants.RobotInfo;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.hopper.HopperSubsystem;
import frc.robot.subsystems.scoring.intake.IntakeSubsystem;
import frc.robot.subsystems.scoring.intake.pivot.PivotState;
import frc.robot.subsystems.scoring.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.AimUtil;
import frc.robot.util.AllianceChooser;
import frc.robot.util.AutoHelper;

import static frc.robot.constants.Controls.OperatorControls;

@SuppressWarnings({"FieldCanBeLocal", "unused"})
public class RobotContainer {
    public static final AllianceChooser allianceChooser = new AllianceChooser();
    public static OI oi;
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final ClimberSubsystem climber;
    private final HopperSubsystem hopper;
    private final ScoringSubsystem scoring;
    private final LEDStrip ledStrip;
    private final SendableChooser<Command> autos;


    public RobotContainer() {
        InterpolatingTables.initializeTables();
        oi = new OI();
        ledStrip = SubsystemManager.getLedStrip();

        swerveDriveSubsystem = SubsystemManager.getSwerveDrive();

        shooter = ShooterSubsystem.getInstance();
        intake = IntakeSubsystem.getInstance();
        hopper = HopperSubsystem.getInstance();
        climber = ClimberSubsystem.getInstance();
        scoring = ScoringSubsystem.getInstance();


        nameCommands();

        autos = new SendableChooser<>();
        autos.setDefaultOption("null auto", new WaitCommand(1));
        autos.addOption("preloaded", scoring.manualSpeakerCommand());
//        for (Command i : AutoFactory.getAutos()) {
//            autos.addOption(i.getName(), i);
//        }
        SmartDashboard.putData("Autons", autos);

        allianceChooser.addSelfToSmartDashboardWithName("Alliance");

        configureBindings();
    }

    private void nameCommands() {
        //climber commands
        NamedCommands.registerCommand("ExtendClimberCommand", climber.runWithSpeedsCommand(1, 1));
        NamedCommands.registerCommand("ManualClimbCommand", climber.runWithSpeedsCommand(0, 0));
        NamedCommands.registerCommand("RetractClimberCommand", climber.runWithSpeedsCommand(-1, -1));
        //intake commands
        NamedCommands.registerCommand("IntakeCommand", Commands.deadline(new WaitCommand(3), scoring.intakeCommand()));
        NamedCommands.registerCommand("OuttakeCommand", scoring.outtakeCommand());
        NamedCommands.registerCommand("ExtendIntake", intake.pivotCommand(PivotState.EXTENDED));
        NamedCommands.registerCommand("RetractIntake", intake.pivotCommand(PivotState.RETRACTED));

        // shooting commands
        NamedCommands.registerCommand("ShootSpeaker", scoring.autoSpeakerCommand());
        NamedCommands.registerCommand("ShootAmp", intake.ampCommand());
        NamedCommands.registerCommand("ManualSpeaker", scoring.manualSpeakerCommand());
    }


    private void configureBindings() {
//
        swerveDriveSubsystem.setDefaultCommand(
            swerveDriveSubsystem.driveFieldCentricCommand()
        );

        DriverControls.ShootOnTheFlyButton
            .whileTrue(
                swerveDriveSubsystem.shootOnTheFlyCommand()
            );

        DriverControls.AmpAlignButton
            .whileTrue(
                Commands.parallel(
                        swerveDriveSubsystem.pathfindCommand(
                            new Pose2d(AimUtil.getAmpPose(), AimUtil.getAmpRotation())
                        ),
                        AutoHelper.ampPrepCommand()
                    )
                    .andThen(intake.ampCommand())
            );

        DriverControls.AimButton
            .whileTrue(
                Commands.parallel(
                    swerveDriveSubsystem.pathfindCommand(
                        AimUtil.getManualSpeakerPose()
                    ),
                    Commands.waitUntil(
                        Controls.canShootOnTheFly
                    ).andThen(
                        scoring.autoSpeakerCommand()
                    )
                )
            );


        // TODO: extract to named class
        ledStrip.setDefaultCommand(new RunCommand(() -> ledStrip.usePattern(
            hopper.hasNote()
                ? new PhasingLEDPattern(new Color8Bit(255, 50, 0), 3)
                : new SolidLEDPattern(new Color8Bit(0, 0, 255))
        ), ledStrip));

        DriverControls.ClimberExtendButton.whileTrue(climber.runWithSpeedsCommand(1, 1));
        DriverControls.ClimberRetractButton.whileTrue(climber.runWithSpeedsCommand(-1, -1));
        DriverControls.ClimberSwap1Button.whileTrue(climber.runWithSpeedsCommand(1, -1));
        DriverControls.ClimberSwap2Button.whileTrue(climber.runWithSpeedsCommand(-1, 1));

        new ConditionalCommand(new RunCommand(() -> {}), hopper.runHopperCommand(RobotInfo.HopperInfo.INTAKE_HOPPER_SPEED), hopper::hasNote);

        OperatorControls.IntakeButton.whileTrue(
            new ParallelCommandGroup(
                intake.intake(),
                hopper.runHopperCommand(RobotInfo.HopperInfo.INTAKE_HOPPER_SPEED)
            ).onlyWhile(hopper::isClear)
        );

        OperatorControls.OuttakeButton.whileTrue(intake.outtakeCommand()
                .alongWith(hopper.runHopperCommand(-RobotInfo.HopperInfo.INTAKE_HOPPER_SPEED)))
            .onFalse((Commands.runOnce(hopper::stopHopper)));

        OperatorControls.IntakeExtendButton.onTrue(intake.pivotCommand(PivotState.EXTENDED));

        OperatorControls.IntakeRetractButton.onTrue(intake.pivotCommand(PivotState.RETRACTED));

        OperatorControls.RunSpeakerShooterButton.whileTrue(scoring.autoSpeakerCommand());
        OperatorControls.RunAmpShooterButton.whileTrue(intake.pivotCommand(PivotState.AMP).andThen(intake.ampCommand())).onFalse(intake.pivotCommand(PivotState.RETRACTED));
        OperatorControls.ManualShooterButton.whileTrue(scoring.manualSpeakerCommand());
        OperatorControls.LaunchShooterButton.whileTrue(scoring.launchCommand());

        OperatorControls.ToggleIR.whileTrue(hopper.toggleIrEnabled());

        DriverControls.ResetGyroButton1.and(DriverControls.ResetGyroButton2)
            .whileTrue(Commands.runOnce(swerveDriveSubsystem::resetOrientation));
    }

    public Command getAutonomousCommand() {
        return autos.getSelected();
    }
}