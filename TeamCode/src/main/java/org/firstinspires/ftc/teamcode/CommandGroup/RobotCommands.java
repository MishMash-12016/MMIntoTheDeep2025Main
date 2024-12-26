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

    private static final int timeIntakeClawClose = 200;
    private static final int timeScoringClaw = 200;
    private static final int timeintakeClose = 100;
    private static final int timeScoringArm = 350;


    /*
    sample intake command -
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
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.intakeSamplePose),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw());


    }
    /* specimen intake
    1. prepare scoring: scoring arm down,scoring, open claw
    2. prepare to intake: intake arm speciman ,rotator, open claw
     */

    public static Command prepareSpecimenIntake() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.transferHold),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.specimanIntake),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.intakeSpecimanPose),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw());
    }

    /* intake sample done  -
    1. intake go down
    2. makes sure claw closed
    3. intake arm up,
    4. close linear intake
    5. make sure elevator down
    6. open+close scoring claw
    7. open intake claw
    8. scoring mid pose
    */
    public static Command IntakeDoneCommand() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.intakePose),
                new WaitCommand(timeintakeClose),
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(timeIntakeClawClose),
                //move the angle of claw to prepare to transfer
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.transferPose),
                new WaitCommand(timeintakeClose),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.holdpose),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.closedPose),
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.elevatorDown),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.transferHold),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(timeScoringClaw),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(timeScoringClaw),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.midPose)
        );

    }

    /* prepare score sample-
    1. make sure scoring claw is closed
    2. elevator get to desired position
    3. scoring arm
     */
    public static Command PrepareHighSample() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(timeScoringClaw),
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.HIGH_BASKET),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoreSampleHigh));
    }

    public static Command PrepareLowSample() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(timeScoringClaw),
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.LOW_BASKET),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoreSampleLow)
        );

    }

    /* prepare score speciman-
    1. make sure scoring claw is closed
    2. elevator get to desired position
    3. scoring arm
     */
    public static Command prepareSpecimenScore() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(timeIntakeClawClose),
                        MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.highChamber),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoreSpecimen)

        );
    }

    /* score speciman-
    1. elevater down a little
    5. scoring claw open
    6. down elevator/down scoring
     */
    public static Command ScoreSpecimen() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.highChamber),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(timeScoringClaw),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.transferHold),
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.elevatorDown)

            );
    }

     /* score sample-
    1. open scoring claw
    2. scoring arm down //wait command?
    3. elevator down
     */

    public static Command ScoreSample() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(timeScoringClaw),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.transferHold),
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.elevatorDown)

                );
    }

    /* fold systems-
    1. elevator down
    2. scoring arm down
    3. linear intake closed
    4. intake arm up
    5. intake rotator hold pose
    6. intake open
     */

    public static Command FoldSystems() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.elevatorDown),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.transferHold),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.closedPose),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.up),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.holdpose),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
        ); }
        public static Command EjectSampleCommand() {
            return new SequentialCommandGroup(
                    MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoreSpecimen),
                    new WaitCommand(timeScoringArm),
                    MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                    new WaitCommand(timeScoringClaw),
                    MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.transferHold)
            );
        }



}


