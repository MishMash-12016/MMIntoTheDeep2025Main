package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.CommandGroup.RobotCommands;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

public class TeleopGeneral extends MMOpMode {
    public TeleopGeneral(OpModeType.NonCompetition opModeType) {
        super(opModeType);
    }

    @Override
    public void onInit() {
        MMRobot.getInstance().mmSystems.initRobotSystems();
        Trigger scoreSample = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.X)
        );

        Trigger totalIntake = new Trigger(
                ()-> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.Y));

        //score high basket- x
        scoreSample.whenActive(
                RobotCommands.ScoreSample()
        );
        scoreSample.whenInactive(
                RobotCommands.HoldPosScoring()
        );

        // intake using button - y
        totalIntake.whileActiveContinuous(
                RobotCommands.IntakeCommand()
        );
        totalIntake.whenInactive(
                RobotCommands.IntakeDoneCommand()
        );


    }
    @Override
    public void run() {
        super.run();
    }
}
