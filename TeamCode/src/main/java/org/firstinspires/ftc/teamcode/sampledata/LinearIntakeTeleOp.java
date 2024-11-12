package org.firstinspires.ftc.teamcode.sampledata;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

import java.util.jar.Attributes;

@TeleOp
public class LinearIntakeTeleOp extends MMOpMode {
    MMRobot robotInstance = MMRobot.getInstance();
    private final double maxOpening = 0.23;

    public LinearIntakeTeleOp(){
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance.mmSystems.initRobotSystems();
        Trigger leftTriggerCondition = new Trigger(
                () -> robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05
        );
        leftTriggerCondition.whenActive(
                robotInstance.mmSystems.linearIntake.setPositionByJoystick(()-> gamepad1.left_trigger*maxOpening));


        waitForStart();

    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Joystick Position: ", robotInstance.mmSystems.linearIntake.getPosition());
        telemetry.update();
    }

}
