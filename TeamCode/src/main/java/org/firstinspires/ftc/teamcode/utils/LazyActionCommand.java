package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.Collections;
import java.util.Set;
import java.util.function.Supplier;

public class LazyActionCommand implements Command {
    private final Supplier<Action> action;
    private Action _action;
    private final Set<Subsystem> requirements;
    private boolean finished = false;

    private Action getAction() {
        return _action == null ? _action = action.get() : _action;
    }

    public LazyActionCommand(Supplier<Action> action) {
        this.action = action;
        this.requirements = Collections.emptySet();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        getAction().preview(packet.fieldOverlay());
        finished = !getAction().run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}