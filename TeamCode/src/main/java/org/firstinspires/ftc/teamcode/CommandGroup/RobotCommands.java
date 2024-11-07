package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;

import java.util.Collections;
import java.util.Set;

public class RobotCommands {

    private static MMSystems mmSystems = MMRobot.getInstance().mmSystems;

    // intake down and open the intake claw
    public static Command IntakeCommand(){
        return new ParallelCommandGroup(
            mmSystems.intakeArm.intakeDown(),
            mmSystems.intakEndUnit.openIntakeClaw());
    }
    //intake close and then intake up
    public static Command IntakeDoneCommand(){
        return new SequentialCommandGroup(
                mmSystems.intakEndUnit.closeIntakeClaw(),
                mmSystems.intakeArm.intakeUp());
    }


    }

