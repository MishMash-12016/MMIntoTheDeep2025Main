package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.TeleOps;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.utils.OpModeType;

@Disabled
@TeleOp
public class TestingFTCBridge extends MMTeleOp {

    MMRobot mmRobot = MMRobot.getInstance();

    public TestingFTCBridge() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    /**
     * this class was mainly just having fun with the libraries i'll be honest :p
     * you can try it yourself! its fun i swear
     */
    @Override
    public void onInit() {

        CuttleMotor motor = mmRobot.mmSystems.controlHub.getMotor(3);

        addRunnableOnInit(
                () -> motor.setPower(-1)
        );

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> motor.setPower(0.5))
        );

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> motor.setPower(0))
        );

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> motor.setPower(-0.5))
        );

    }

}
