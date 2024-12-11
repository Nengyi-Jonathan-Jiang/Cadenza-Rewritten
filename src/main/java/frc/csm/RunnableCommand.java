package frc.csm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BooleanSupplier;

/**
 * A command that also implements the {@link Runnable} interface. The {@link Runnable#run() run}
 * method shall run the command as if scheduled (by calling {@link Command#initialize() initialize},
 * {@link Command#execute() execute}, {@link Command#isFinished() isFinished}, and
 * {@link Command#end(boolean) end}). This can be dangerous for commands that execute forever or
 * until a condition is met as it will block the thread. The class also provides a
 * {@link RunnableCommand#runUntil(BooleanSupplier) runUntil} method, which runs the command until
 * it finishes or until a condition is met, upon which it will interrupt the command and return.
 */
public abstract class RunnableCommand extends Command implements Runnable {
    @Override
    public void run() {
        runUntil(() -> false);
    }

    public void runUntil(BooleanSupplier shouldInterrupt) {
        initialize();
        while(!isFinished()) {
            if(shouldInterrupt.getAsBoolean()) {
                end(true);
                return;
            }
            execute();
        }
        end(false);
    }

    public static RunnableCommand wrap(Runnable runnable, Subsystem... requirements) {
        return wrap(new InstantCommand(runnable, requirements));
    }

    public static RunnableCommand wrap(Command c) {
        return new RunnableCommand(){
            {
                addRequirements(c.getRequirements().toArray(Subsystem[]::new));
            }

            @Override
            public void initialize() {
                c.initialize();
            }

            @Override
            public void execute() {
                c.execute();
            }

            @Override
            public void end(boolean interrupted) {
                c.end(interrupted);
            }

            @Override
            public boolean isFinished() {
                return c.isFinished();
            }
        };
    }
}
