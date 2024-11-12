package org.firstinspires.ftc.teamcode.TeleOp;

import android.widget.Button;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.CommandGroup.RobotCommands;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

public class TeleOp extends MMOpMode {
    private GamepadEx gamepadEx;
    public TeleOp(OpModeType.NonCompetition opModeType) {
        super(opModeType);
    }

    @Override
    public void onInit() {
        MMRobot.getInstance().mmSystems.initRobotSystems();
        gamepadEx = new GamepadEx(gamepad1);

        //button x-
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.X)){
            RobotCommands.ScoreSample();
            RobotCommands.HoldPosScoring();
        }


        // button y -total intake
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.Y)){
            RobotCommands.IntakeCommand();
            RobotCommands.IntakeDoneCommand();
        }
        waitForStart();
    }
    @Override
    public void run() {
        super.run();
    }
    }

