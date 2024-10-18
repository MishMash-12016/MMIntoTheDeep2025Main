package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.TeleOps;

import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Commands.ShootBySupplier;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

import java.util.function.BooleanSupplier;


@Disabled
@TeleOp(name = "Experimenting")
public class ExperimentingTeleOp extends MMTeleOp {

    MMRobot mmRobot = MMRobot.getInstance();

    public ExperimentingTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    boolean isActive = false;

    BooleanSupplier isActiveSupplier = this::getIsActive;

    public boolean getIsActive() {
        return isActive;
    }

    @Override
    public void onInit() {
        mmRobot.mmSystems.shooter = new Shooter();

        Runnable changeStage = () -> isActive = !isActiveSupplier.getAsBoolean();

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).toggleWhenActive(
                new ShootBySupplier(mmRobot.mmSystems.gamepadEx1::getRightY).alongWith(
                        new StartEndCommand(
                                changeStage,
                                changeStage
                        )
                )
        );

    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("isActive", isActiveSupplier.getAsBoolean());
        telemetry.update();
    }
}
