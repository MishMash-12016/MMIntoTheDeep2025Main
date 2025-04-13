package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;


public class ClimbingCommand {
    static boolean goUp = true;

    public static Command PrepareClimbToThird() {
        return new ConditionalCommand(
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.ELEVATOR_HIGH_CHAMBER),
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.ELEVATOR_HIGH_CHAMBER.position.get()/2),
                () -> (goUp = !goUp)
        );
    }

    public static Command ClimbToThird() {
        return new SequentialCommandGroup(
                new RunCommand(() -> MMRobot.getInstance().mmSystems.elevator.setPower(-1.0), MMRobot.getInstance().mmSystems.elevator)
                        .interruptOn(() -> MMRobot.getInstance().mmSystems.elevator.getHeight() < Elevator.ElevatorState.ELEVATOR_CLIMB.position.get()),
                new InstantCommand(() -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0), MMRobot.getInstance().mmSystems.elevator)
        );
    }
}
