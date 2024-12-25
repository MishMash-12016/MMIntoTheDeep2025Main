package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;

import java.util.function.DoubleSupplier;

public class RobotCommands {

    private static final int timeClawClose = 200;

    /*
    intake command -
    1. open linear intake
    2. prepare scoring: scoring arm down,scoring, open claw
    3. prepare to intake: intake arm partly down,rotator, open claw
    */

    public static Command IntakeCommand(DoubleSupplier intakeTrigger) {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.linearIntake.setPositionByJoystick(intakeTrigger),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.transferHold),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.prepareSampleIntake),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.intakePose),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw());


    }
    /* specimen intake
    open claw
    elevator down
    prepare scoring angle
     */

    public static Command prepareSpecimenIntake() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.prepareSpecimanIntake)
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
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.intakePose),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
                //move the angle of claw to prepare to transfer
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.transferPose),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.holdpose),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.closedPose),
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.elevatorDown),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.transferHold),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(450),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.midPose),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.closedPose)

        );

    }

    public static Command EjectSampleCommand() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoreSpecimen),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.transferHold)

        );
    }

    /* score sample-
    1. elevator get to desired height
    2. scoring servo turn
    3. scoring claw open
     */
    public static Command PrepareHighSample() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.HIGH_BASKET),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoreSampleHigh));
    }

    public static Command PrepareLowSample() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.LOW_BASKET),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoreSampleLow),
                new WaitCommand(300)
        );

    }

    public static Command prepareSpecimenScore() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(timeClawClose),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.highChamber),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoreSpecimen)
                )
        );
    }

    public static Command ScoreSpecimen() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.highChamberScorePose),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoreSampleHigh),
                        MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.elevatorDown)

                ));
}

public static Command ScoreSample() {
    return new SequentialCommandGroup(
            MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
            new WaitCommand(200),
            new ParallelCommandGroup(
                    new WaitCommand(1000),
                    MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.elevatorDown),
                    MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.transferHold)
            ));
}

public static Command FoldSystems() {
    return new ParallelCommandGroup(
            MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.elevatorDown),
            MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.transferHold),
            MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.closedPose),
            MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.up),
            MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.holdpose),
            MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
    );
}


}


