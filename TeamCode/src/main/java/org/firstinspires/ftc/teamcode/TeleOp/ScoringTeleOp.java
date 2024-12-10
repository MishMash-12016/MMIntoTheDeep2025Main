package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.RobotCommands;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class ScoringTeleOp extends MMOpMode {
    MMRobot robotInstance = MMRobot.getInstance();
    Trigger rightTriggerCondition;
    public ScoringTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }
    @Override
    public void onInit() {
        robotInstance.mmSystems.initRobotSystems();
        rightTriggerCondition = new Trigger(
                () -> robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05
        );
        rightTriggerCondition.whileActiveOnce(
                robotInstance.mmSystems.scoringClawEndUnit.closeScoringClaw()

        );
        rightTriggerCondition.whenInactive(
                robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.scoreSample).andThen(
                        new WaitCommand(500).andThen(
                                robotInstance.mmSystems.scoringClawEndUnit.openScoringClaw()
                        ))

        );
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("trigger",rightTriggerCondition.get());
        telemetry.update();
    }

}


