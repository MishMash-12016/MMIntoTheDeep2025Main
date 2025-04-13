package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm.IntakeArmState;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator.IntakeRotatorState;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm.ScoringArmState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;

public class IntakeSpecimenCommand {

    public static Command SpecimenIntake() {
        return new SequentialCommandGroup(
//                new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.intakeDistSensor.getDistance() < 4),
                //intake
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                new WaitCommand(100),

                //score
                new ParallelCommandGroup(
                        new ScoringArm_SpeedControll(300,ScoringArmState.SCORING_ARM_SCORE_POSE.position.get()),
                        new SequentialCommandGroup(
                                new WaitCommand(50),
                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(0.35),
                                new WaitCommand(50),
                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SPECIMEN_POSE)
                        )
                )
        );
    }


    public static Command SpecimenIntakeAuto() {
        return new SequentialCommandGroup(
                //intake
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(0),

                //score
                new ParallelCommandGroup(
                        new ScoringArm_SpeedControll(500,ScoringArmState.SCORING_ARM_SCORE_POSE.position.get()),
                        new SequentialCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(0.35),
                                new WaitCommand(50),
                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SPECIMEN_POSE)
                        )
                )
        );
    }


    public static Command PrepareSpecimenIntakeFront() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeRotatorState.INIT_POSE),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.INIT_POSE)
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.INTAKE_FROM_FRONT_POSE),
                        new ScoringArm_SpeedControll(400,ScoringArmState.INTAKE_FROM_FRONT_POSE.position.get()),
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                )
        );

//        return new ConditionalCommand(
//                new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
//                                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeRotatorState.INIT_POSE),
//                                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.INIT_POSE)
//                        ),
//                        new WaitCommand(200),
//                        new ParallelCommandGroup(
//                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.INTAKE_FROM_FRONT_POSE),
//                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.INTAKE_FROM_FRONT_POSE),
//                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
//                        )
//                ),
//                new ParallelCommandGroup(
//                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.INTAKE_FROM_FRONT_POSE),
//                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.INTAKE_FROM_FRONT_POSE),
//                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
//                ),
//                () -> MMRobot.getInstance().mmSystems.intakeArm.estimatedPose >= IntakeArmState.SPECIMEN_INTAKE.position.get()
//        );
    }
}