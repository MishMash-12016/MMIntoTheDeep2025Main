package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm.IntakeArmState;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator.IntakeRotatorState;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake.LinearIntakeState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm.ScoringArmState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator.ScoringRotatorState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotatorYAxis;
import org.firstinspires.ftc.teamcode.SubSystems.Wisher;

import java.util.function.BooleanSupplier;

public class IntakeSampleCommand {
    public static Command prepareSampleIntake(BooleanSupplier rotateRightButton,BooleanSupplier rotateLeftButton) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.maxOpening),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeRotatorState.INTAKE_SAMPLE_POSE),
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                ),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.REST_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotatorYAxis.setPosition(ScoringEndUnitRotatorYAxis.ScoringRotatorYAxisState.SAMPLE_TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.REST_POSE),//be prepared for transfer
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                )
        );
    }
    public static Command prepareSampleIntake() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
        );
    }

    public static Command prepareSampleIntake_Lime() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
        );
    }

    public static Command SampleIntake() {
        return new SequentialCommandGroup(
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.INTAKE_POSE), //collect pose
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(180),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.MID_INTAKE_SPECIMEN),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.REST_POSE),
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.CLOSED_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotatorYAxis.setPosition(ScoringEndUnitRotatorYAxis.ScoringRotatorYAxisState.SAMPLE_TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.REST_POSE),//be prepared for transfer
                        new WaitCommand(30).andThen(
                                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.8)
                        )
                )
        );
    }
    public static Command SampleInakeAndPrepare(){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.maxOpening),
                        new ConditionalCommand(
                                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                                new InstantCommand(),
                                ()-> MMRobot.getInstance().mmSystems.intakeDistSensor.getDistance() < 0.5
                        ),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.PREPARE_TRANSFER),
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                ),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.INTAKE_POSE), //collect pose
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.CLOSED_POSE),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeRotatorState.INTAKE_SAMPLE_POSE),
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.TRANSFER_POSE)
                )
                //not finished
        );
    }
    public static Command Transfer(){
        return  new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.TRANSFER_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.TRANSFER_POSE),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(500),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.TRANSFER_POSE),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INTAKE_SAMPLE_POSE),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                //MMRobot.getInstance().mmSystems.elevator.ElevatorGetToZero(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(500),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.MID_POSE)

        );
    }


}
