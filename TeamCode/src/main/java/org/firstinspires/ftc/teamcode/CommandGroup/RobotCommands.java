package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;

public class RobotCommands {

    private static MMSystems mmSystems = MMRobot.getInstance().mmSystems;

    // intake down and open the intake claw
    public static Command IntakeCommand() {
        return new ParallelCommandGroup(
                mmSystems.intakeArm.intakeDown(),
                mmSystems.intakEndUnit.openClaw());
    }

    //intake close and then intake up
    public static Command IntakeDoneCommand() {
        return new SequentialCommandGroup(
                mmSystems.intakEndUnit.closeClaw(),
                mmSystems.intakeArm.intakeUp());
    }

}

