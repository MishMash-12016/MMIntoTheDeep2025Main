package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class LinearIntakeTeleOp extends MMOpMode {
    MMRobot robotInstance = MMRobot.getInstance();
    private final double maxOpening = 1;
    Trigger rightJoystickCondition;

    public LinearIntakeTeleOp(){
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance.mmSystems.initRobotSystems();
        rightJoystickCondition = new Trigger(
                () -> robotInstance.mmSystems.gamepadEx2.getRightY() > 0.05
        );
        Trigger rightCondition= new Trigger(
                ()->  robotInstance.mmSystems.gamepadEx2.getButton(GamepadKeys.Button.A)
        );
        rightJoystickCondition.whileActiveOnce(
                robotInstance.mmSystems.linearIntake.setPositionByJoystick(()-> gamepad2.right_stick_y));
        rightCondition.whenActive(
                robotInstance.mmSystems.linearIntake.setPosition(0)
        );


    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("trigger", rightJoystickCondition.get());
        telemetry.addData("Joystick Position: ", robotInstance.mmSystems.linearIntake.getPosition());
        telemetry.update();
    }

}
