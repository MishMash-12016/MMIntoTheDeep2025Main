package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.CommandGroup.RobotCommands;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnit;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

public class TeleopGeneral extends MMOpMode {
    public TeleopGeneral() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {
        MMRobot.getInstance().mmSystems.initRobotSystems();

        Trigger prepareHighSampleTrigger = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.X)
        );

        Trigger scoreHigheSampleTrigger = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.Y)
        );

        Trigger IntakeTrigger = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05);

        //button that prepares score- x
        scoreHigheSampleTrigger.whileActiveOnce(RobotCommands.PrepareHighSample());

        //new button that score- y
        prepareHighSampleTrigger.whileActiveOnce(MMRobot.getInstance().mmSystems.scoringEndUnit.openScoringClaw());

        // intake using trigger- right
        IntakeTrigger.whileActiveOnce(
                RobotCommands.IntakeCommand(() -> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)).whenFinished(
                        () -> RobotCommands.IntakeDoneCommand().schedule()));


    }

    @Override
    public void run() {
        super.run();
    }
}
