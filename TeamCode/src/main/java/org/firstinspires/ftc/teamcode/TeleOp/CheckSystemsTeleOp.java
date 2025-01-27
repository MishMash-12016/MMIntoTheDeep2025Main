package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
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
                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.TRANSFER_POSE),
                new WaitCommand(300),
                robotInstance.mmSystems.intakeArm.setPosition(0.2),
                new WaitCommand(500),
                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                new WaitCommand(300),
                robotInstance.mmSystems.linearIntake.setPosition(0.1)
                )
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(()->robotInstance.mmSystems.elevator.setPower(3.0)).withTimeout(300),
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
                        new InstantCommand(()->robotInstance.mmSystems.elevator.setPower(-3.0)).withTimeout(300)
                        )
        );
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}
