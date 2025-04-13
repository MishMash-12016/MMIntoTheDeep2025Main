package org.firstinspires.ftc.teamcode.CommandGroup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.Autonomous.AutoOnePlusFiveRightRed;
import org.firstinspires.ftc.teamcode.Autonomous.ActionCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm.IntakeArmState;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake.LinearIntakeState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm.ScoringArmState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.utils.FixedSequentialCommandGroup;

import java.util.function.BooleanSupplier;

public class IntakeSampleCommand {
    public static ElapsedTime elapsedTime = new ElapsedTime();

    public static Command prepareSampleIntake(BooleanSupplier rotateRightButton, BooleanSupplier rotateLeftButton) {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE),//be prepared for transfer
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.MAX_OPENING),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.rotateByButton(rotateLeftButton, rotateRightButton),
                new WaitCommand(120).andThen(
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                )
        );
    }

    public static Command prepareSampleIntakeWithoutButton() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.MAX_OPENING),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),

                new WaitCommand(110).andThen(
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                ));
    }
    public static Command SampleIntake() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.SAMPLE_INTAKE_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(0.34),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INIT_POSE),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.CLOSED_POSE)
                )
        );
    }
    public static Command SampleIntakeWithoutRequirments() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPositionWithoutRequirments(IntakeArmState.SAMPLE_INTAKE_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPositionWithoutRequirments(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                MMRobot.getInstance().mmSystems.scoringArm.setPositionWithoutRequirments(ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakEndUnit.setPoseWithoutRequirments(IntakEndUnit.IntakeClawState.CLOSE.position.get()),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPositionWithoutRequirments(IntakeArmState.SPECIMEN_INTAKE),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPositionWithoutRequirments(IntakeEndUnitRotator.IntakeRotatorState.INIT_POSE.position.get()),
                        MMRobot.getInstance().mmSystems.linearIntake.setPositionWithoutRequirments(LinearIntakeState.CLOSED_POSE)
                )
        );
    }

    public static Command limeLightIntake_TeleOp() {
        return new FixedSequentialCommandGroup(
                new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.switchToDetector()),
                new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.vision.getPipelineIndex() == Vision.currentPipeline),

                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.SPECIMEN_INTAKE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE)
                ),

                //Lamlam side:
                new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.setPreviousResult()),
                MMRobot.getInstance().mmSystems.vision.angleChange(),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),
                limelightGetter.strafeToSample(),

                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.MAX_OPENING),
                new WaitCommand(400),
                FirstSampleIntake()
        ).interruptOn(
                () -> MMRobot.getInstance().mmSystems.driveTrain.joystickMoved()).andThen(MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.CLOSED_POSE));
    }

    public static Command limeLightIntake_Auto() {
        return new FixedSequentialCommandGroup(
                new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.switchToDetector()),
                new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.vision.getPipelineIndex() == Vision.currentPipeline),

                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.SPECIMEN_INTAKE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE)
                ),

                //Lamlam side:
                new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.setPreviousResult()),
                MMRobot.getInstance().mmSystems.vision.angleChange(),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),
                limelightGetter.strafeToSampleAuto());
    }

    public static Command limeLightIntake_Auto_for_specimen() {
        return new FixedSequentialCommandGroup(
                new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.vision.switchToDetector()),
                new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.vision.getPipelineIndex() == Vision.currentPipeline),


                new FixedSequentialCommandGroup(
                        new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.setPreviousResult()),
                        MMRobot.getInstance().mmSystems.vision.angleChange(),
                        new ParallelCommandGroup(
                                limelightGetter.strafeToSample(),
                                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.MAX_OPENING),
                                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE))
                ),

                new SequentialCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SAMPLE_INTAKE_POSE),
                        new WaitCommand(300),
                        MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                MMRobot.getInstance().mmSystems.intakeArm.setPosition(0.34),
                                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE)
                        )
                )
        );
    }
    public static ActionCommand driveRight(){
        return new ActionCommand(
                MMRobot.getInstance().mmSystems.driveTrain.actionBuilder(MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR())
                        .strafeTo(new Vector2d(MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR().position.x,MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR().position.y + 5 ))
                        .build());
    }

    public static Command doLimelightIntake_Auto(double angle) {
        return new FixedSequentialCommandGroup(
                new ConditionalCommand(
                                driveRight(),
                        new InstantCommand(()->{}),
                        ()->MMRobot.getInstance().mmSystems.vision.isTargetVisible()
                ),
                new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.switchToDetector()),
                new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.vision.getPipelineIndex() == Vision.currentPipeline),

                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE)
                ),

                //Lamlam side:
                new FixedSequentialCommandGroup(
                        new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.setPreviousResult()),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(angle),

                        limelightGetter.strafeToSample().alongWith(
                                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.MAX_OPENING))

//                        limelightGetter.getOpenLinearToSample()
                ).interruptOn(
                        () -> MMRobot.getInstance().mmSystems.driveTrain.joystickMoved()),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),
                new WaitCommand(200),
                FirstSampleIntake(),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.CLOSED_POSE)
        );
    }




    public static Command FirstSampleIntake() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SAMPLE_INTAKE_POSE),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INIT_POSE),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE)
                )
        );
    }
}