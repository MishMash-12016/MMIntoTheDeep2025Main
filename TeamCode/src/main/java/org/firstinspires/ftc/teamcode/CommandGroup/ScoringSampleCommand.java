package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator.ElevatorState;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm.ScoringArmState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;

public class ScoringSampleCommand {
    public static Command PrepareHighSampleEran(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.elevator.moveToPose(ElevatorState.ELEVATOR_DOWN),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SAMPLE_TRANSFER_POSE)
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SAMPLE_POSE)
                ),
                new WaitCommand(50),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SAMPLE_TRANSFER_POSE),
                new WaitCommand(50),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SCORING_ARM_SCORE_POSE),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE)
        );
    }

    public static Command PrepareHighSample(){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.elevator.ElevatorGetToZeroSensor(),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SAMPLE_TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SAMPLE_POSE)
                ),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SAMPLE_TRANSFER_POSE),
                new WaitCommand(50),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SCORING_ARM_SCORE_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_SCORE),
                new WaitCommand(100),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.elevator.getHeight() > 43),
                        MMRobot.getInstance().mmSystems.elevator.moveToPose(ElevatorState.HIGH_BASKET).alongWith(
                                new WaitCommand(200).andThen(
                                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE)
                                )
                        ),
                        new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.elevator.getHeight() > 37).andThen(
                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE)
                        )
                ),
                new InstantCommand(() -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0), MMRobot.getInstance().mmSystems.elevator)
        );
    }

    public static Command PrepareHighSample_Auto(){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SAMPLE_TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SAMPLE_POSE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SAMPLE_TRANSFER_POSE)
                ),
                new WaitCommand(50),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                new WaitCommand(100),

                new WaitCommand(100),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SCORING_ARM_SCORE_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_SCORE),
                        MMRobot.getInstance().mmSystems.elevator.moveToPose(ElevatorState.HIGH_BASKET),
                        new WaitCommand(300).andThen(
                                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE)
                        )
                ),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE)
        );
    }

    public static Command ScoreHighSample(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(150),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.elevator.ElevatorGetToZeroSensor(),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER)
                )
        );
    }

    public static Command CheatScoreHighSample(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(0.1),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.elevator.ElevatorGetToZeroSensor(),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER)
                )
        );
    }

    public static Command PrepareLowSample(){
        return new SequentialCommandGroup(
                //MMRobot.getInstance().mmSystems.elevator.moveToPose(ElevatorState.ELEVATOR_DOWN),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SAMPLE_TRANSFER_POSE)
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SAMPLE_POSE)
                ),
                new WaitCommand(50),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SAMPLE_TRANSFER_POSE),
                new WaitCommand(50),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SCORING_ARM_SCORE_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_SCORE),
                new WaitCommand(100),
                //MMRobot.getInstance().mmSystems.elevator.moveToPose(ElevatorState.HIGH_BASKET),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE)
        );
    }
    public static Command ScoreLowSample() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SCORING_ARM_SCORE_POSE),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SAMPLE_TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER)
                )
        );
    }
}