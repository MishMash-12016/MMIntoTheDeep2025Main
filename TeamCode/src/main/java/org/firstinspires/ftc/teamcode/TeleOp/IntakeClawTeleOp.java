package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakEndUnit;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class IntakeClawTeleOp extends MMOpMode {
    MMRobot robotInstance = MMRobot.getInstance();
    Trigger rightTriggerCondition;
    public IntakeClawTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }
    @Override
    public void onInit() {
        robotInstance.mmSystems.initRobotSystems();
        rightTriggerCondition = new Trigger(
                () -> robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05
        );
        rightTriggerCondition.whileActiveOnce(
                robotInstance.mmSystems.intakEndUnit.openIntakeClaw(IntakEndUnit.open)
        );
        rightTriggerCondition.whenInactive(
                robotInstance.mmSystems.intakEndUnit.closeIntakeClaw(IntakEndUnit.close)
        );
}

    @Override
    public void run() {
        super.run();
        telemetry.addData("trigger",rightTriggerCondition.get() );
        telemetry.addData("Joystick Position: ", robotInstance.mmSystems.intakEndUnit.getPosition());
        telemetry.update();
    }

}
