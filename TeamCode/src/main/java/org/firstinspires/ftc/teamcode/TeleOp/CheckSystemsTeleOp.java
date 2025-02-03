package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.message.redux.ReceiveGamepadState;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

import java.util.function.DoubleSupplier;

@TeleOp
public class CheckSystemsTeleOp extends MMOpMode {
    MMRobot robotInstance = MMRobot.getInstance();
    public CheckSystemsTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {
        robotInstance.mmSystems.initRobotSystems();
        robotInstance.mmSystems.initDriveTrain();
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(
                robotInstance.mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.ROTATE_LEFT_ANGLE),
                new WaitCommand(300),
                robotInstance.mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.ROTATE_RIGHT_ANGLE),
                new WaitCommand(500),
                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.TRANSFER_POSE),
                new WaitCommand(300),
                robotInstance.mmSystems.intakeArm.setPosition(0.2),
                new WaitCommand(500),
                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                new WaitCommand(300),
                robotInstance.mmSystems.linearIntake.setPosition(0.2)
                )
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.LOW_BASKET),
                        new WaitCommand(500),
                        robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.INIT_POSE),
                        new WaitCommand(300),
                        robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.MID_POSE),
                        new WaitCommand(500),
                        robotInstance.mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.CLOSE),
                        new WaitCommand(300),
                        robotInstance.mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.OPEN),
                        new WaitCommand(500),
                        robotInstance.mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORE_SAMPLE_POSE),
                        new WaitCommand(300),
                        robotInstance.mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.MID_POSE_SPECIMEN),
                        MMRobot.getInstance().mmSystems.elevator.moveToPose(10))

        );


        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INTAKE_SAMPLE_POSE)
                )
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new SequentialCommandGroup(
                        robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.maxOpening),
                        robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                        robotInstance.mmSystems.intakeEndUnitRotator.setPosition(0.65),
                        robotInstance.mmSystems.intakEndUnit.setPose(0.68))
        );


//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
//                new ParallelCommandGroup(
//                        robotInstance.mmSystems.linearIntake.setPosition(new DoubleSupplier() {
//                            @Override
//                            public double getAsDouble() {
//                                return 0.6;
//                            }
//                        }),
//                        robotInstance.mmSystems.intakeEndUnitRotator.setPosition(0),
//                        robotInstance.mmSystems.intakEndUnit.setPose(0.7),
//                        robotInstance.mmSystems.intakeArm.setPosition(0.58))
//        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                        robotInstance.mmSystems.intakeArm.setPosition(0.6)

        );


        /*MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw()
                )
        );*/

        /*
        new Trigger(() -> robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05) //slow mode
                .whileActiveContinuous(
                        () -> MMRobot.getInstance().mmSystems.elevator.setPower(robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))
                );
        new Trigger(() -> robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05) //slow mode
                .whileActiveContinuous(
                        () -> MMRobot.getInstance().mmSystems.elevator.setPower(-robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))
                );*/



        //MMRobot.getInstance().mmSystems.elevator.setPower(robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))

        /*MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(scoringPos),
                        new InstantCommand(() -> scoringPos -= 0.01)
                )
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(intakePos),
                        new InstantCommand(() -> intakePos += 0.01)
                )
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new SequentialCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(intakePos),
                        new InstantCommand(() -> intakePos -= 0.01)
                )
        );*/
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("height:", robotInstance.mmSystems.elevator.getHeight());
        telemetry.addData("right:", robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        telemetry.addData("left:", -robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        telemetry.update();
    }
}
