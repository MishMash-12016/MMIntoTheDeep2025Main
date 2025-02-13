package org.firstinspires.ftc.teamcode.Libraries.MMLib.DriveTrain.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.MMRobot;

public class ResetFieldOrientedCommand extends InstantCommand {
    public ResetFieldOrientedCommand() {
        super(() -> MMRobot.getInstance().mmSystems.localizer.setYawScalar(
                MMRobot.getInstance().mmSystems.localizer.getHeading()));
    }
}
