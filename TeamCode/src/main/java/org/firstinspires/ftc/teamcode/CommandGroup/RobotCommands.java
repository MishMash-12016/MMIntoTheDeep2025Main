package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;

import java.util.function.DoubleSupplier;

public class RobotCommands {
    private static final  MMSystems mmSystems= MMRobot.getInstance().mmSystems;
    private static final double linearIntakeClosed = 0;
    private static final int timeClawClose = 200;
    private static final int timeClawOpen = 100;
    public final static double elevatorDown = 0;
    public final static int timeScoringArm = 150;

    /* intake receive -
    1. open linear intake
    2. intake down
    3. open claw */

    public static Command IntakeCommand(DoubleSupplier intakeTrigger) {
        return new ParallelCommandGroup(
                mmSystems.linearIntake.setPositionByJoystick(intakeTrigger),
                mmSystems.intakeArm.intakeDown(),
                mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.inatkepose),
                mmSystems.intakEndUnit.openIntakeClaw());


    }
    /* specimen intake
    open claw
    elevator down
    prepare scoring angle
     */

    public static Command prepareSpecimenIntake() {
        return new ParallelCommandGroup(
                mmSystems.scoringEndUnit.openScoringClaw(),
                mmSystems.elevator.moveToPose(Elevator.elevatorWallHeight),
                mmSystems.scoringEndUnit.scoreSpecimen()
        );
    }

    /* intake done  -
    1. close claw
    2. intake arm up  ,
    3. close linear intake
    4. elevator down
    5. open+close scoring claw
    6. open intake claw
    */
    public static Command IntakeDoneCommand() {
        return new SequentialCommandGroup(
                mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(timeClawClose),
                new ParallelCommandGroup(
                        //move the angle of claw to prepare to transfer
                        mmSystems.intakeArm.intakeUp(),
                        mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.holdpose),
                        mmSystems.linearIntake.setPosition(linearIntakeClosed),
                        mmSystems.elevator.moveToPose(elevatorDown),
                        mmSystems.scoringEndUnit.scoringArmHold(),
                        mmSystems.scoringEndUnit.openScoringClaw()
                ), mmSystems.scoringEndUnit.closeScoringClaw(),
                mmSystems.intakEndUnit.openIntakeClaw()

        );

    }

    public static Command EjectSampleCommand() {
        return new SequentialCommandGroup(
                mmSystems.scoringEndUnit.scoreSample(),
                new WaitCommand(timeScoringArm),
                mmSystems.scoringEndUnit.openScoringClaw(),
                new WaitCommand(timeClawOpen),
                mmSystems.scoringEndUnit.scoringArmHold()

        );
    }

    /* score sample-
    1. elevator get to desired height
    2. scoring servo turn
    3. scoring claw open
     */
    public static Command PrepareHighSample() {
        return new ParallelCommandGroup(
                mmSystems.elevator.moveToPose(Elevator.HIGH_BASKET),
                mmSystems.scoringEndUnit.scoreSample());
    }

    public static Command PrepareLowSample() {
        return new ParallelCommandGroup(
                mmSystems.elevator.moveToPose(Elevator.LOW_BASKET),
                mmSystems.scoringEndUnit.scoreSample());
    }

    public static Command prepareSpecimenScore() {
        return new SequentialCommandGroup(
                mmSystems.scoringEndUnit.closeScoringClaw(),
                new WaitCommand(timeClawClose),
                new ParallelCommandGroup(
                        mmSystems.elevator.moveToPose(Elevator.highChamber),
                        mmSystems.scoringEndUnit.scoreSpecimen()
                )
        );
    }

    public static Command ScoreSpecimen() {
        return new SequentialCommandGroup(
                mmSystems.elevator.moveToPose(Elevator.highChamberScorePose),
                mmSystems.scoringEndUnit.openScoringClaw(),
                new ParallelCommandGroup(
                        mmSystems.scoringEndUnit.scoreSample(),
                        mmSystems.elevator.moveToPose(Elevator.elevatorDown)

                ));
}

public static Command ScoreSample() {
    return new SequentialCommandGroup(
            mmSystems.scoringEndUnit.openScoringClaw(),
            new WaitCommand(timeClawOpen),
            new ParallelCommandGroup(
                    mmSystems.elevator.moveToPose(Elevator.elevatorDown),
                    mmSystems.scoringEndUnit.scoringArmHold()
            ));
}

public static Command FoldSystems() {
    return new ParallelCommandGroup(
            mmSystems.elevator.moveToPose(Elevator.elevatorDown),
            mmSystems.scoringEndUnit.scoringArmHold(),
            mmSystems.linearIntake.setPosition(linearIntakeClosed),
            mmSystems.intakeArm.intakeUp(),
            mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.holdpose),
            mmSystems.intakEndUnit.openIntakeClaw()
    );
}


}


