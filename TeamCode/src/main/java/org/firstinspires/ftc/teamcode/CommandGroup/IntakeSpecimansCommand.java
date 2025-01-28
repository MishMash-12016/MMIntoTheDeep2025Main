package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator.ElevatorState;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm.IntakeArmState;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator.IntakeRotatorState;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake.LinearIntakeState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm.ScoringArmState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator.ScoringRotatorState;

public class IntakeSpecimansCommand {
    public static Command PrepareSpecimanIntake(){
        return new SequentialCommandGroup(
            MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.PREPARE_TRANSFER),//be prepared for transfer
            MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.SCORE_SAMPLE_POSE),
            MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
            MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.SPECIMEN_INTAKE),
            MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeRotatorState.INTAKE_SPECIMEN_POSE),
            MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.TRANSFER_POSE),
            MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw());
    }
    public static Command SpecimenIntake(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.PREPARE_TRANSFER),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.SPECIMEN_INTAKE),//make sure your their
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.MID_INTAKE_SPECIMEN),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeRotatorState.HOLD_POSE_SPECIMEN),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.TRANSFER_POSE),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.TRANSFER_SPECIMEN_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.TRANSFER_POSE),
                //MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE), //makes sure
                //        MMRobot.getInstance().mmSystems.elevator.moveToPose(ElevatorState.ELEVATOR_DOWN),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(), //makes sure
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.MID_POSE)

        );
    }
}
