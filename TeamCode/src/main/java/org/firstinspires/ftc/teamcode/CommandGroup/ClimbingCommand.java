package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

public class ClimbingCommand {
    public static Command PrepareClimbToThird() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.ELEVATOR_LOW_CHAMBER)
        );
    }

    public static Command ClimbToThird() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> MMRobot.getInstance().mmSystems.elevator.setPower(1.0)),
                new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.elevator.getHeight() < 5),
                MMRobot.getInstance().mmSystems.hook.OpenHook()
        );
    }
}
