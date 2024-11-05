package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;

public class DriveCommand extends RunCommand {

    public DriveCommand() {
        super(
                () -> MMRobot.getInstance().mmSystems.driveTrain.fieldOrientedDrive(
                        -MMRobot.getInstance().mmSystems.gamepadEx1.getLeftX(),
                        -MMRobot.getInstance().mmSystems.gamepadEx1.getLeftY(),
                        -MMRobot.getInstance().mmSystems.gamepadEx1.getRightX()
                ),
                MMRobot.getInstance().mmSystems.driveTrain
        );
    }
}
