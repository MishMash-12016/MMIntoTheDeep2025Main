package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.RobotCommands;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class IntakeAndScoringTeleOp extends MMOpMode {
    MMRobot robotInstance = MMRobot.getInstance();
    Trigger rightTriggerCondition;
    Trigger leftTriggerCondition;
    public IntakeAndScoringTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }
    @Override
    public void onInit() {
        robotInstance.mmSystems.initRobotSystems();
        rightTriggerCondition = new Trigger(
                () -> robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05
        );
        leftTriggerCondition = new Trigger(
                () -> robotInstance.mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05
        );
        Trigger closeclawCondition = new Trigger(
                ()-> robotInstance.mmSystems.gamepadEx1.getButton(GamepadKeys.Button.A)
        );

        leftTriggerCondition.whileActiveOnce(
                RobotCommands.PrepareHighSample()
        );
        rightTriggerCondition.whileActiveOnce(
                RobotCommands.IntakeCommand(()-> gamepad1.right_trigger)

        );
        closeclawCondition.whenActive(
                        RobotCommands.IntakeDoneCommand()
        );
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("trigger",rightTriggerCondition.get());
        telemetry.update();
    }

}

