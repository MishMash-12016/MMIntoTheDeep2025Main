package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.TeleOps;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMUtils;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.utils.OpModeType;

@TeleOp
public class CartridgeTest extends MMTeleOp {
    public CartridgeTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    MMRobot mmRobot = MMRobot.getInstance();

    @Override
    public void onInit() {
        CuttleServo servo = mmRobot.mmSystems.expansionHub.getServo(4);

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whileActiveOnce(
                new CommandBase() {
                    @Override
                    public void execute() {
                        servo.setPosition(
                                1 - MMUtils.joystickToServo( //reversed
                                        mmRobot.mmSystems.gamepadEx1.getLeftX()
                                )
                        );
                    }
                }
        );

    }
}
