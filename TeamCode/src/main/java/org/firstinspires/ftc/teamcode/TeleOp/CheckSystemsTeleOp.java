package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

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

                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
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
                        MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET),
                        new WaitCommand(500),

                        robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.INIT_POSE),
                        new WaitCommand(300),
                        robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORING_ARM_SCORE_POSE),
                        new WaitCommand(500),

                        robotInstance.mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.CLOSE),
                        new WaitCommand(300),
                        robotInstance.mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.OPEN),
                        new WaitCommand(500),

                        robotInstance.mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE),
                        new WaitCommand(300),
                        robotInstance.mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.INIT_POSE),
                        new WaitCommand(500),

                        MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.ELEVATOR_DOWN))
                );


        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                        robotInstance.mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET)
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                robotInstance.mmSystems.elevator.moveToPose(Elevator.ElevatorState.ELEVATOR_DOWN)
        );


    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("height:", robotInstance.mmSystems.elevator.getHeight());
        telemetry.addData("right:", robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        telemetry.addData("left:", -robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        telemetry.addData("dis:", robotInstance.mmSystems.intakeDistSensor.getDistance());
        telemetry.update();
    }
}
