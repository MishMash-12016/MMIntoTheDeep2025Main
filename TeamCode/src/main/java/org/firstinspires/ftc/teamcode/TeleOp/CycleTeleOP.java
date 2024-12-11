
package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.RobotCommands;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class CycleTeleOP extends MMOpMode {

    public CycleTeleOP() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        MMRobot.getInstance().mmSystems.initRobotSystems();
        Trigger leftTrigger = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.05
        );
        Trigger closeClawCondition= new Trigger(
                ()-> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.A)
        );
        Trigger rightTrigger= new Trigger(
                ()-> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.05
        );
        Trigger scoreCondition = new Trigger(
                ()-> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT)
        );
        rightTrigger.whileActiveOnce(
                RobotCommands.IntakeCommand(()->gamepad1.right_trigger)
        );
        closeClawCondition.whenActive(
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw()
        );
        rightTrigger.whenInactive(
                RobotCommands.IntakeDoneCommand()
        );
        leftTrigger.whenActive(
                RobotCommands.PrepareLowSample()
        );
        scoreCondition.whenActive(
                RobotCommands.ScoreSample()
        );


    }


    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        FtcDashboard.getInstance().getTelemetry().addData("height", MMRobot.getInstance().mmSystems.elevator.getHeight());
        FtcDashboard.getInstance().getTelemetry().addData("right trigger", gamepad1.right_trigger);
        FtcDashboard.getInstance(


        ).getTelemetry().addData("target pose:", MMRobot.getInstance().mmSystems.elevator.targetPose);
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
