package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator.ElevatorState;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm.ScoringArmState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;

public class ScoringSampleCommand {
    public static Command  PrepareScoreHigh(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.MID_POSE),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.CLOSE.position),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.elevator.moveToPose(ElevatorState.HIGH_BASKET), //the height of the high basket
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SCORE_SAMPLE),
                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORE_SAMPLE_POSE)
        );
    }
    public static Command PrepareScoreLow(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.CLOSE.position),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORE_SAMPLE_POSE), 
                MMRobot.getInstance().mmSystems.elevator.moveToPose(ElevatorState.LOW_BASKET), //the height of the low basket
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SCORE_SAMPLE)
        );
    }
    public static Command ScoreSample(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SCORE_SAMPLE),
                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORE_SAMPLE_POSE),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.MID_POSE),
                new WaitCommand(400),
                MMRobot.getInstance().mmSystems.elevator.moveToPose(ElevatorState.ELEVATOR_DOWN)
        );

    }
}
