package org.firstinspires.ftc.teamcode.Libraries.MMLib.Commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMUtils;

import java.util.Set;
import java.util.function.Supplier;

/**
 * Defers {@link Command} construction to runtime. Runs the command returned by the supplier when this
 * command is initialized, and ends when it ends. Useful for performing runtime tasks before
 * creating a new command. If this command is interrupted, it will cancel the command.
 * <p>Note that the supplier <i>must</i> create a new Command each call. For selecting one of a
 * preallocated set of commands, use {@link SelectCommand}.
 */
public class MMDeferredCommand extends CommandBase {

    private final Command nullCommand = new InstantCommand();

    private final Supplier<Command> commandSupplier;
    private Command command = nullCommand;

    /**
     * Creates a new DeferredCommand that runs the supplied command when initialized, and ends when it
     * ends. Useful for lazily creating commands at runtime. The {@link Supplier} will be called each
     * time this command is initialized. The Supplier <i>must</i> create a new Command each call.
     *
     * @param commandSupplier The command supplier
     * @param requireSubsystems The command requirements. This is a {@link Set} to prevent accidental
     *     omission of command requirements. Use {@link java.util.HashSet#add(Object) HashSet.add(Subsystem)} to <s>easily</s>  construct a requirement
     *     set.
     */
    public MMDeferredCommand(Supplier<Command> commandSupplier, Set<Subsystem> requireSubsystems) {
        this.commandSupplier = MMUtils.requireNonNullParam(commandSupplier, "Supplier<Command>", "MMDeferredCommand");
        addRequirements(requireSubsystems.toArray(new Subsystem[0]));
    }

    @Override
    public void initialize() {
        Command cmd = commandSupplier.get();
        if(cmd != null) command = cmd;
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
        command = nullCommand;
    }
}
