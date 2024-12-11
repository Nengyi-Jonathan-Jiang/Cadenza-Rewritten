package frc.lib.tuning;

public interface ValueSource {
    String name();

    default void update() {}
}
