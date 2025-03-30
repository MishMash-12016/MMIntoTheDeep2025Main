package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;


public class ClimbingCommand {
    public static Command PrepareClimbToThird() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.ELEVATOR_LOW_CHAMBER),
                MMRobot.getInstance().mmSystems.hook.OpenHook()
        );
    }

    public static Command ClimbToThird() {
        return new SequentialCommandGroup(
                new RunCommand(() -> MMRobot.getInstance().mmSystems.elevator.setPower(-0.8), MMRobot.getInstance().mmSystems.elevator)
                        .interruptOn(() -> MMRobot.getInstance().mmSystems.elevator.getHeight() < Elevator.ElevatorState.ELEVATOR_CLIMB.position.get()),
                new InstantCommand(() -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0), MMRobot.getInstance().mmSystems.elevator)
        );
    }
}
