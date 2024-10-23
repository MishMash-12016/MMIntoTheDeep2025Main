package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.TeleOps;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMUtils;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Disabled
@TeleOp
public class Mapping extends MMOpMode {

    public Mapping() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    MMRobot mmRobot = MMRobot.getInstance();

    CuttleServo servo;

    @Override
    public void onInit() {

        servo = mmRobot.mmSystems.controlHub.getServo(0);

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whileActiveOnce(
                new CommandBase() {
                    @Override
                    public void execute() {
                        servo.setPosition(
                                MMUtils.joystickToServo(
                                        mmRobot.mmSystems.gamepadEx1.getLeftX()
                                )
                        );
                    }
                }
        );

    }
}
