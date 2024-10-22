package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.Intake;
import org.firstinspires.ftc.teamcode.Commands.ClawSetState;
import org.firstinspires.ftc.teamcode.Commands.LinearIntakeCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;@TeleOp
public class OnlyFTesting extends MMTeleOp {
    public OnlyFTesting() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }



    @Override
    public void onInit() {
        MMRobot.getInstance().mmSystems.initScoringArm();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.scoringArm.setPosition(gamepad1.left_trigger);
        telemetry.addData("meow",MMRobot.getInstance().mmSystems.scoringArm.getPosition());
        telemetry.update();
    }
}
