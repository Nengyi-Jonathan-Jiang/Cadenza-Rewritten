package frc.robot.subsystems.scoring.hopper;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.csm.PackagePrivate;
import frc.robot.constants.IDs;

@PackagePrivate
class PhysicalHopperSubsystem extends HopperSubsystem {
    private final CANSparkMax hopperMotor;
    private final DigitalInput irSensor;

    private final Debouncer irDebouncer = new Debouncer(0.25);
    private boolean noteDetected = false;

    @PackagePrivate
    PhysicalHopperSubsystem() {
        hopperMotor = new CANSparkMax(IDs.HOPPER_MOTOR, CANSparkMax.MotorType.kBrushed);
        hopperMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        this.irSensor = new DigitalInput(IDs.IR_SENSOR_1_DIO_PORT);
    }

    @Override
    public void runHopper(double speed) {
        hopperMotor.set(speed);
    }

    @Override
    public void stopHopper() {
        hopperMotor.stopMotor();
    }

    protected boolean noteDetected() {
        return noteDetected;
    }

    @Override
    public void periodic() {
        boolean sensorIsClear = irSensor.get();
        boolean rawHasNote = !sensorIsClear;
        this.noteDetected = irDebouncer.calculate(rawHasNote);
        SmartDashboard.putBoolean("Hopper has note", hasNote());
        SmartDashboard.putBoolean("Hopper ir enabled", isIrEnabled());
    }
}
