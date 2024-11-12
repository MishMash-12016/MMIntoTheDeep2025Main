package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

import java.util.Collections;
import java.util.Set;

public class RobotCommands {

    private static MMSystems mmSystems = MMRobot.getInstance().mmSystems;
    private static int timeLinearIntake= 500; //millis
    private static double linearIntakeOpen = 2.22;
    /* intake recieve -
    1. open linear intake
    2. wait  ,
    3. intake down and open claw*/

    public static Command IntakeCommand() {
        return new SequentialCommandGroup(mmSystems.linearIntake.setPosition(linearIntakeOpen),
                new WaitCommand(timeLinearIntake),
                new ParallelCommandGroup(
                mmSystems.intakeArm.intakeDown(),
                mmSystems.intakEndUnit.openIntakeClaw()), new WaitCommand(2*timeLinearIntake));


    }

    /* intake done  -
    1. close claw
    2. intake arm up  ,
    3. close linear intake */
    public static Command IntakeDoneCommand() {
        return new SequentialCommandGroup(
                mmSystems.intakEndUnit.closeIntakeClaw(),
                mmSystems.intakeArm.intakeUp()).
                andThen(mmSystems.linearIntake.setPosition(0));
    }

    /* score sample-
    1. elevator get to desired height
    2. scoring servo turn
    3. scoring claw open
     */
    public static Command ScoreSample(){
        return new SequentialCommandGroup(
                mmSystems.elevator.moveToPose(Elevator.HIGH_BASKET),
                mmSystems.scoringEndUnit.scoreScoringServo(),
                mmSystems.scoringEndUnit.openScoringClaw(),new WaitCommand(timeLinearIntake));
    }
    /* scoring back to hold-
    1. scoring claw close
    2. elevator go back to desired height and scoring servo turn
     */
    public static Command HoldPosScoring(){
        return new SequentialCommandGroup(mmSystems.scoringEndUnit.closeScoringClaw(),
                new ParallelCommandGroup(
                        mmSystems.elevator.moveToPose(Elevator.elevatorDown),
                        mmSystems.scoringEndUnit.holdScoringServo()));
    }
}

