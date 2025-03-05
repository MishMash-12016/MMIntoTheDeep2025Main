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
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.maxOpening),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.rotateByButton(rotateLeftButton,rotateRightButton),
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                ),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.REST_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SAMPLE_TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.REST_POSE),//be prepared for transfer
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                )
        );
    }

//    public static Command SampleIntake() {
//        return new SequentialCommandGroup(
//                new WaitCommand(200),
//                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.INTAKE_POSE), //collect pose
//                new WaitCommand(200),
//                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
//                new WaitCommand(180),
//                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.MID_INTAKE_SPECIMEN),
//                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
//                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.CLOSED_POSE),
//                MMRobot.getInstance().mmSystems.scoringEndUnitRotatorYAxis.setPosition(ScoringEndUnitRotatorYAxis.ScoringRotatorYAxisState.SAMPLE_TRANSFER_POSE),
//                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.REST_POSE),//be prepared for transfer
////                        new WaitCommand(30).andThen(
////                                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.8)
////                        )),
//                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(0.8),
//                new WaitCommand(2000),
//                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.MID_POSE_SPECIMEN)
//        );
//    }

    public static Command SampleIntake(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.INTAKE_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.REST_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SAMPLE_TRANSFER_POSE),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.REST_POSE),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.SAMPLE_TRANSFER_POSE),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.CLOSED_POSE),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SAMPLE_POSE)


        );
    }
}
