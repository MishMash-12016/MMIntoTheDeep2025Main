package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm.ScoringArmState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator.ScoringRotatorState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotatorYAxis;

public class AutoSpecimensCommand {
    public static Command SpecimenScorePreLoad(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORE_SPECIMEN_POSE),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.scoringEndUnitRotatorYAxis.setPosition(ScoringEndUnitRotatorYAxis.ScoringRotatorYAxisState.SCORE_POSE),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORE_SPECIMEN)
        );

    }

    public static Command SpecimenIntakeAuto() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.PREPARE_TRANSFER),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE)),//make sure your their
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(0.3),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.MID_INTAKE_SPECIMEN),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.TRANSFER_POSE),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.HOLD_POSE_SPECIMEN),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),//makes sure
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.TRANSFER_POSE)),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.TRANSFER_POSE),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE), //makes sure
                //        MMRobot.getInstance().mmSystems.elevator.moveToPose(ElevatorState.ELEVATOR_DOWN),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.MID_POSE_SPECIMEN),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(0.4),
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotatorYAxis.setPosition(ScoringEndUnitRotatorYAxis.ScoringRotatorYAxisState.SCORE_POSE),
                        new WaitCommand(200).andThen(MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.SCORE_SPECIMEN_POSE))
                )
        );
    }


    }
