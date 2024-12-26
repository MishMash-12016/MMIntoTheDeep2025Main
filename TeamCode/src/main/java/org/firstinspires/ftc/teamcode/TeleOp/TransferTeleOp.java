package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.utils.OpModeType;
@TeleOp
public class TransferTeleOp extends MMOpMode {
    MMRobot robotInstance = MMRobot.getInstance();
    Trigger rightTriggerCondition;
    public TransferTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }
    @Override
    public void onInit() {
        robotInstance.mmSystems.initRobotSystems();
        rightTriggerCondition = new Trigger(
                () -> robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05
        );
        Trigger randbuttonCondition = new Trigger(
                ()-> robotInstance.mmSystems.gamepadEx1.getButton(GamepadKeys.Button.A)
        );
        rightTriggerCondition.whileActiveOnce(
                new SequentialCommandGroup(
                        robotInstance.mmSystems.intakEndUnit.openIntakeClaw(),
                        robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.intakePose),
                        robotInstance.mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.intakeSamplePose)
                        ));
        randbuttonCondition.whenActive(
                        robotInstance.mmSystems.intakEndUnit.closeIntakeClaw());
        rightTriggerCondition.whenInactive(
                new SequentialCommandGroup(
                        robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.transferHold),
                        robotInstance.mmSystems.scoringClawEndUnit.openScoringClaw(),
                        new WaitCommand(200),
                        robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.closedPose),
                        new WaitCommand(200),
                        robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.up),
                        robotInstance.mmSystems.scoringClawEndUnit.closeScoringClaw(),
                        new WaitCommand(1000),
                        robotInstance.mmSystems.intakEndUnit.openIntakeClaw(),
                        new WaitCommand(1000),
                        robotInstance.mmSystems.scoringArm.setPosition(0.8),
                        robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.closedPose)
                ));
    }
    @Override
    public void run() {
        super.run();
        telemetry.addData("trigger",rightTriggerCondition.get() );
        telemetry.update();
    }

}

