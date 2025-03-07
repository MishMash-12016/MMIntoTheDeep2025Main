package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm.IntakeArmState;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake.LinearIntakeState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm.ScoringArmState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;

import java.util.function.BooleanSupplier;

public class IntakeSampleCommand {
    public static Command prepareSampleIntake(BooleanSupplier rotateRightButton,BooleanSupplier rotateLeftButton) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.MAX_OPENING),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.rotateByButton(rotateLeftButton,rotateRightButton),
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                ),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.REST_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SAMPLE_TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SAMPLE_TRANSFER_POSE),//be prepared for transfer
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                )
        );
    }

    public static Command SampleIntake(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.SAMPLE_INTAKE_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SAMPLE_TRANSFER_POSE),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SAMPLE_TRANSFER_POSE),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.SAMPLE_TRANSFER_POSE),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.CLOSED_POSE),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SAMPLE_POSE),
                new WaitCommand(150),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE)


        );
    }
}
