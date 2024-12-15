package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;


@TeleOp
public class ScoringEndUnitTeleOp extends MMOpMode {
    MMRobot robotInstance;

    public ScoringEndUnitTeleOp(){
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();

        robotInstance.mmSystems.initRobotSystems();
        Trigger buttonTrigger = new Trigger(
                () -> robotInstance.mmSystems.gamepadEx2.getButton(GamepadKeys.Button.A)
        );

        Trigger buttonTrigger1 = new Trigger(
                () -> robotInstance.mmSystems.gamepadEx2.getButton(GamepadKeys.Button.B)
        );

        buttonTrigger.whileActiveOnce(
                robotInstance.mmSystems.scoringClawEndUnit.openScoringClaw());

        buttonTrigger1.whileActiveOnce(
                robotInstance.mmSystems.scoringClawEndUnit.closeScoringClaw());
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Joystick Position: ", robotInstance.mmSystems.linearIntake.getPosition());
        telemetry.update();
    }

    }

