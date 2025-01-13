package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class ScoringRotaterTeleOP extends MMOpMode {
    MMRobot robotInstance = MMRobot.getInstance();
    public ScoringRotaterTeleOP(){
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }
    @Override
    public void onInit() {
        robotInstance.mmSystems.initRobotSystems();
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                robotInstance.mmSystems.scoringEndUnitRotator.setPosition(0)
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                robotInstance.mmSystems.scoringEndUnitRotator.setPosition(1)
        );

    }

    @Override
    public void run() {
        super.run();

    }


}


