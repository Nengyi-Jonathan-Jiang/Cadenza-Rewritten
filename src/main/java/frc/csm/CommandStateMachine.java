package frc.csm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

public class CommandStateMachine extends Command {

    public final State finishedState = new FinishedState(this);
    private final Collection<State> allStates = new ArrayList<>(List.of(finishedState));
    private State initialState = null;
    private boolean isSetupComplete = false;
    private State currentState = null;
    private boolean isExecuting = false;
    private double executionStartTimeSeconds = 0;

    @PackagePrivate
    void assertSetupNotComplete() {
        if (isSetupComplete) {
            throw new RuntimeException(
                "Cannot configure state machine after finalizeSetup() has been called"
            );
        }
    }

    @PackagePrivate
    void assertSetupComplete() {
        if (!isSetupComplete) {
            throw new RuntimeException(
                "Must call finalizeSetup() before scheduling the CSM as a command"
            );
        }
    }

    @PackagePrivate
    void assertHasState(State state) {
        if (!allStates.contains(state)) {
            throw new RuntimeException(
                "Can only reference states from the same state machine"
            );
        }
    }

    // State machine execution variables and functions

    /**
     * Adds a new state from the given commands to the state machine. It is an error
     * to call this after {@link #finalizeSetup()} has been called.
     */
    public State addState(CommandSupplier... commands) {
        assertSetupNotComplete();
        State result = new State(this);

        for (CommandSupplier c : commands) {
            result.addCommand(c);
        }

        if (initialState == null) {
            initialState = result;
        }
        allStates.add(result);
        return result;
    }

    /**
     * Sets the initial state of the state machine. It is an error to call this after
     * {@link #finalizeSetup()} has been called.
     */
    public void setInitial(State initialState) {
        assertSetupNotComplete();
        assertHasState(initialState);
        this.initialState = initialState;
    }

    /**
     * Prevents the state machine from being further configured. This MUST be called
     * before the command is scheduled.
     */
    public void finalizeSetup() {
        assertSetupNotComplete();

        addRequirements(
            allStates.stream()
                .map(State::getRequirements)
                .flatMap(Collection::stream)
                .collect(Collectors.toSet())
                .toArray(Subsystem[]::new)
        );

        isSetupComplete = true;
    }

    @PackagePrivate
    void assertIsExecuting() {
        assertSetupComplete();
        if (!isExecuting) {
            throw new RuntimeException(
                "Cannot use state machine execution functions when state machine is not executing"
            );
        }
    }

    /**
     * Returns the elapsed time since the command state machine started execution, in seconds
     */
    public double elapsedTime() {
        assertIsExecuting();
        return Timer.getFPGATimestamp() - executionStartTimeSeconds;
    }

    // Command methods

    @Override
    public void initialize() {
        assertSetupComplete();

        this.currentState = Objects.requireNonNullElse(
            initialState,
            finishedState
        );

        this.isExecuting = true;
        this.executionStartTimeSeconds = Timer.getFPGATimestamp();
        this.currentState.enterState();
    }

    private StateTransition getTransitionFromCurrentState() {
        for (StateTransition transition : this.currentState.getTransitions()) {
            if (transition.condition().get()) {
                return transition;
            }
        }
        return null;
    }

    @Override
    public void execute() {
        this.currentState.executeState();

        // Check for transitions
        StateTransition transition = getTransitionFromCurrentState();
        if (transition != null) {
            State nextState = transition.nextState();

            currentState.exitState();
            transition.action().run();
            nextState.enterState();

            currentState = nextState;
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.currentState.exitState();
        this.isExecuting = false;
    }

    @Override
    public boolean isFinished() {
        return currentState == finishedState;
    }
}
