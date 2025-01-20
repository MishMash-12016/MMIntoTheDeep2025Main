package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSpecimanCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class IntakeArmTeleOp extends MMOpMode {
    MMRobot robotInstance = MMRobot.getInstance();
    public IntakeArmTeleOp(){
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }
    @Override
    public void onInit() {
        robotInstance.mmSystems.initRobotSystems();
        robotInstance.mmSystems.initDriveTrain();
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                robotInstance.mmSystems.scoringArm.setPosition(0.3)
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                robotInstance.mmSystems.scoringArm.setPosition(0.7)
        );
//        if (MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0) {
//            MMRobot.getInstance().mmSystems.scoringArm.setPosition(
//                    MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)).schedule();
//        }
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
//                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.MID_POSE)
//        );
//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                MMRobot.getInstance().mmSystems.scoringArm.setPosition(0)
//        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                ScoringSpecimanCommand.SpecimanScore()
        );
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
        telemetry.update();
    }


}
