package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm.IntakeArmState;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator.IntakeRotatorState;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm.ScoringArmState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator.ScoringRotatorState;

import java.util.function.BooleanSupplier;


public class IntakeSpecimenCommand {
    public static Command PrepareSystemsSpecimenIntake() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeRotatorState.DEFAULT_POSE),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SPECIMEN_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.SPECIMEN_TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SPECIMEN_TRANSFER_POSE)//be prepared for transfer



        );
    }

    public static Command PrepareSpecimenIntakeWithSensor(BooleanSupplier press) {
        return new SequentialCommandGroup(
                PrepareSystemsSpecimenIntake(),
                new RunCommand(() -> {
                    if (press.getAsBoolean())
                        CommandScheduler.getInstance().cancelAll();
                }),
                new WaitCommand(200),
                new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.intakeDistSensor.getDistance() < 3.8),//3.8
                SpecimenIntake()
        );
    }

    public static Command SpecimenIntake() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                    MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SPECIMEN_TRANSFER_POSE),
                    MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SPECIMEN_POSE),
                    MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.SPECIMEN_TRANSFER_POSE),
                    MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.SPECIMEN_INTAKE),
                    MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeRotatorState.DEFAULT_POSE),
                    MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                    MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE)
                ),
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
//                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.MID_INTAKE_SPECIMEN),
//                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.TRANSFER_SPECIMEN_POSE),
                new WaitCommand(250),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeRotatorState.DEFAULT_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.SAMPLE_TRANSFER_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SPECIMEN_POSE),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORE_SPECIMEN),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.MID_POSE)
        );
    }

    public static Command PrepareSystemsSpecimenIntakeFromBack() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.INTAKE_FROM_BACK_POSE),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.SCORING_POSE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.INTAKE_FROM_BACK_POSE),
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                )
        );
    }

    public static Command SpecimenIntakeFromBack() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw()
        );
    }
}
