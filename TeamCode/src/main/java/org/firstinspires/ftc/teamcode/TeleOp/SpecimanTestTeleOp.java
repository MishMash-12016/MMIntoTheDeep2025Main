package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimansCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSpecimanCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class SpecimanTestTeleOp extends MMOpMode {
    MMRobot robotInstance = MMRobot.getInstance();
    public SpecimanTestTeleOp(){
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }
    @Override
    public void onInit() {
        robotInstance.mmSystems.initRobotSystems();
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                IntakeSpecimansCommand.SpecimenIntake()
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                IntakeSpecimansCommand.PrepareSpecimanIntake()
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0)
        );
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
//                MMRobot.getInstance().mmSystems.linearIntake.setPosition(0.1));
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                MMRobot.getInstance().mmSystems.linearIntake.setPosition(1));

//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                ScoringSpecimanCommand.PrepareSpecimanScore()
//        );
//
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                ScoringSpecimanCommand.SpecimanScore()
//        );
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
//                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
//        );
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
//                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw()
//        );
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(0)
//        );
    }

    @Override
    public void run() {
        super.run();

    }


}
