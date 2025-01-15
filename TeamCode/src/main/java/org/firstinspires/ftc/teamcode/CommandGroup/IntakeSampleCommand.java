package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm.IntakeArmState;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake.LinearIntakeState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm.ScoringArmState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator.ScoringRotatorState;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeSampleCommand {

    public static Command prepareSampleIntake(DoubleSupplier trigger, BooleanSupplier bool) {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(trigger),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.rotateByButton(bool)
        );
    }

    public static Command CloseClawBYsensorTest() {

        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(500),
        !MMRobot.getInstance().mmSystems.intakeDistSensor.checkDis()?
         MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw() : MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw()
        );
    }

    public static Command SampleIntake() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.INTAKE_POSE), //collect pose
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.TRANSFER_POSE),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.TRANSFER_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.HOLD_POSE),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.HOLD_POSE_SPECIMEN),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.OPEN.position),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.CLOSED_POSE),
               // MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.ELEVATOR_DOWN),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(600),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.MID_POSE)
        );
    }

    public static Command sampleEject() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SCORE_SAMPLE),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
        );
    }


}
