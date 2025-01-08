package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimansCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSpecimanCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
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
        robotInstance.mmSystems.initDriveTrain();
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                IntakeSpecimansCommand.SpecimenIntake()
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                IntakeSpecimansCommand.PrepareSpecimanIntake()
        );
//        if (MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0) {
//            MMRobot.getInstance().mmSystems.scoringArm.setPosition(
//                    MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)).schedule();
//        }

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                ScoringSpecimanCommand.PrepareSpecimanScore()
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw()
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.barelyopen)
        );


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
