package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.oi.OI;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.util.DriverStationUtil;

public final class Controls {
    private static final OI oi = OI.getInstance();

    public static Trigger inShootingRange = new Trigger(SubsystemManager.getSwerveDrive()::inShootingRange);
    public static Trigger aligned = new Trigger(SubsystemManager.getSwerveDrive()::isAligned);
    public static Trigger inShootingSector = new Trigger(SubsystemManager.getSwerveDrive()::inShootingSector);
    public static Trigger canShootOnTheFly = inShootingRange.and(aligned).and(inShootingSector);
    public static Trigger spinupTrigger = new Trigger(
        SubsystemManager.getSwerveDrive()::inSpinupRange
    )
        .and(DriverControls.ShootOnTheFlyButton.and(canShootOnTheFly).negate());
    public static double rumbleStrength = 0.5;

    public static final class DriverControls {
        public static final Trigger AimButton = oi.driverController().LEFT_TRIGGER;
        public static final Trigger ShootOnTheFlyButton = oi.driverController().RIGHT_TRIGGER;
        public static final Trigger ClimberExtendButton = oi.driverController().POV_UP;
        public static final Trigger ClimberRetractButton = oi.driverController().POV_DOWN;
        public static final Trigger ClimberSwap1Button = oi.driverController().POV_LEFT;
        public static final Trigger ClimberSwap2Button = oi.driverController().POV_RIGHT;
        public static final Trigger AmpAlignButton = oi.driverController().X_BUTTON;
        public static final Trigger ResetGyroButton1 = oi.driverController().A_BUTTON;
        public static final Trigger ResetGyroButton2 = oi.driverController().B_BUTTON;

        public static double getSwerveForwardAxis() {
            return (DriverStationUtil.isRed() ? -1 : 1)
                * oi.driverController().leftStickY()
                * RobotInfo.SwerveInfo.CURRENT_MAX_ROBOT_MPS;
        }

        public static double getSwerveStrafeAxis() {
            return (DriverStationUtil.isRed() ? 1 : -1)
                * oi.driverController().leftStickX()
                * RobotInfo.SwerveInfo.CURRENT_MAX_ROBOT_MPS;
        }

        public static double getSwerveRotationAxis() {
            return (DriverStationUtil.isRed() ? 1 : -1)
                * oi.driverController().rightStickX()
                * RobotInfo.SwerveInfo.CURRENT_MAX_ROBOT_MPS;
        }
    }

    public static final class OperatorControls {
        public static final Trigger RunSpeakerShooterButton = oi.operatorController().RIGHT_TRIGGER;
        public static final Trigger RunAmpShooterButton = oi.operatorController().LEFT_BUMPER;
        public static final Trigger ManualShooterButton = oi.operatorController().LEFT_TRIGGER;

        public static final Trigger IntakeButton = oi.operatorController().X_BUTTON;
        public static final Trigger OuttakeButton = oi.operatorController().Y_BUTTON;
        public static final Trigger IntakeExtendButton = oi.operatorController().POV_DOWN;
        public static final Trigger IntakeRetractButton = oi.operatorController().POV_UP;

        public static final Trigger ToggleIR = oi.operatorController().A_BUTTON;

        public static final Trigger LaunchShooterButton = oi.operatorController().RIGHT_BUMPER;
        public static final Trigger FeedShooterButton = oi.operatorController().POV_LEFT;
    }
}
