package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator.ElevatorState;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm.ScoringArmState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;

public class ScoringSampleCommand {
    public static Command PrepareHighSample(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.ELEVATOR_DOWN),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.8),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SAMPLE_TRANSFER_POSE),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SAMPLE_POSE)
                ),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SAMPLE_TRANSFER_POSE),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.MID_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORE_POSE)
                ),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.elevator.moveToPose(ElevatorState.HIGH_BASKET),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.PREPARE_SCORE_SAMPLE.position.get()+0.05),
                new WaitCommand(50),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.PREPARE_SCORE_SAMPLE)
        );
    }
    public static Command  ScoreHighSample(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.MID_POSE),
                new WaitCommand(400),
                MMRobot.getInstance().mmSystems.elevator.ElevatorGetToZeroSensor(),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.PREPARE_TRANSFER),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SPECIMEN_POSE)
        );
    }
}