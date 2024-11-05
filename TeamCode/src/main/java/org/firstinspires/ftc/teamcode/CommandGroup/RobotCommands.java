package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;

public class RobotCommands {

    private static MMSystems mmSystems = MMRobot.getInstance().mmSystems;

    public static Command IntakeCommand(){
        return new ParallelCommandGroup(
            mmSystems.intakeArm.intakeDown(),
            mmSystems.intakEndUnit.setIntakePower());
    }
}
