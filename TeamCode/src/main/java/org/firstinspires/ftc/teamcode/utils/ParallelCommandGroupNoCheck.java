package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandGroupBase;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class ParallelCommandGroupNoCheck extends CommandGroupBase {

    // maps commands in this group to whether they are still running
    private final Map<Command, Boolean> m_commands = new HashMap<>();
    private boolean m_runWhenDisabled = true;

    /**
     * Creates a new ParallelCommandGroup.  The given commands will be executed simultaneously.
     * The command group will finish when the last command finishes.  If the CommandGroup is
     * interrupted, only the commands that are still running will be interrupted.
     *
     * @param commands the commands to include in this group.
     */
    public ParallelCommandGroupNoCheck(Command... commands) {
        addCommands(commands);
    }

    @Override
    public final void addCommands(Command... commands) {
        requireUngrouped(commands);

        if (m_commands.containsValue(true)) {
            throw new IllegalStateException(
                    "Commands cannot be added to a CommandGroup while the group is running");
        }

        for (Command command : commands) {
            m_commands.put(command, false);
            m_runWhenDisabled &= command.runsWhenDisabled();
        }
    }

    @Override
    public void initialize() {
        for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
            commandRunning.getKey().initialize();
            commandRunning.setValue(true);
        }
    }

    @Override
    public void execute() {
        for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
            if (!commandRunning.getValue()) {
                continue;
            }
            commandRunning.getKey().execute();
            if (commandRunning.getKey().isFinished()) {
                commandRunning.getKey().end(false);
                commandRunning.setValue(false);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
                if (commandRunning.getValue()) {
                    commandRunning.getKey().end(true);
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return !m_commands.values().contains(true);
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_runWhenDisabled;
    }

}
