package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
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
        rightTriggerCondition.whileActiveOnce(
                new SequentialCommandGroup(
                        robotInstance.mmSystems.linearIntakeEndUnitRotator.setPosition(LinearIntakeEndUnitRotator.holdpose),
                        robotInstance.mmSystems.intakEndUnit.closeIntakeClaw()));
        rightTriggerCondition.whenInactive(
                new SequentialCommandGroup(
                        robotInstance.mmSystems.scoringClawEndUnit.closeScoringClaw(),
                        new WaitCommand(1000),
                        robotInstance.mmSystems.intakEndUnit.openIntakeClaw(),
                        new WaitCommand(1000),
                        robotInstance.mmSystems.scoringArm.setPosition(0.8)
                ));
    }
    @Override
    public void run() {
        super.run();
        telemetry.addData("trigger",rightTriggerCondition.get() );
        telemetry.update();
    }

}

