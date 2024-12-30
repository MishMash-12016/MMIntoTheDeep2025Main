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

    public LinearIntakeTeleOp(){
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance.mmSystems.initRobotSystems();

        Trigger aButton= new Trigger(
                ()->  robotInstance.mmSystems.gamepadEx2.getButton(GamepadKeys.Button.A)
        );
        Trigger bButton= new Trigger(
                ()->  robotInstance.mmSystems.gamepadEx2.getButton(GamepadKeys.Button.B)
        );
        bButton.whenActive(robotInstance.mmSystems.linearIntake.setPosition(0.6));

        aButton.whenActive(
                robotInstance.mmSystems.linearIntake.setPosition(0)
        );


    }

    @Override
    public void run() {
        super.run();
        telemetry.addData(" Position: ", robotInstance.mmSystems.linearIntake.getPosition());
        telemetry.update();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        MMRobot.getInstance().mmSystems.controlHub.pullBulkData();
    }

}
