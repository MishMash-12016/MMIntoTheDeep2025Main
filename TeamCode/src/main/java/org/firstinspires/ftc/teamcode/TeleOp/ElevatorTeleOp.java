package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class ElevatorTeleOp extends MMOpMode {

    public ElevatorTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        MMRobot.getInstance().mmSystems.initRobotSystems();
        Trigger leftTriggerCondition = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05
        );
        Trigger rightTriggerCondition = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05
        );
        Trigger gotoCondition = new Trigger(
                ()-> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.A)
        );
        leftTriggerCondition.whenActive(
                MMRobot.getInstance().mmSystems.elevator.setPowerByJoystick(()-> -gamepad1.left_trigger));
        rightTriggerCondition.whenActive(
                MMRobot.getInstance().mmSystems.elevator.setPowerByJoystick(()-> gamepad1.right_trigger));
        gotoCondition.whileActiveOnce(
                MMRobot.getInstance().mmSystems.elevator.moveToPose(700)
        );

    }


    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        FtcDashboard.getInstance().getTelemetry().addData("height", MMRobot.getInstance().mmSystems.elevator.getTicks());
        FtcDashboard.getInstance().getTelemetry().addData("right trigger", gamepad1.right_trigger);
        FtcDashboard.getInstance().getTelemetry().addData("target pose:",MMRobot.getInstance().mmSystems.elevator.targetPose);
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
