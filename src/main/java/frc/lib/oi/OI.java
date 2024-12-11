package frc.lib.oi;

import java.util.Objects;

public class OI {
    private static volatile OI instance;

    public static synchronized OI getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, OI::new);
    }


    private final Controller driverController;
    private final Controller operatorController;

    public OI() {
        driverController = new Controller(0);
        operatorController = new Controller(1);
    }

    public Controller operatorController() {
        return operatorController;
    }

    public Controller driverController() {
        return driverController;
    }
}
