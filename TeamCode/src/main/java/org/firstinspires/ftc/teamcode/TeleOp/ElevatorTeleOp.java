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
public class ElevatorTeleOp extends MMOpMode {

    public ElevatorTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        MMRobot.getInstance().mmSystems.initRobotSystems();
//        Trigger leftTriggerCondition = new Trigger(
//                () -> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05
//        );
//        Trigger rightTriggerCondition = new Trigger(
//                () -> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05
//        );
//        Trigger setPowerCondition = new Trigger(
//                () -> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.A)
//        );
        Trigger gotoPoseCondition = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.B)
        );
//        Trigger gotoPoseCondition2 = new Trigger(
//                () -> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.X)
//        );
//        leftTriggerCondition.whenActive(
//                MMRobot.getInstance().mmSystems.elevator.setPowerByJoystick(() -> -gamepad1.left_trigger));
//        rightTriggerCondition.whenActive(
//                MMRobot.getInstance().mmSystems.elevator.setPowerByJoystick(() -> gamepad1.right_trigger));
        gotoPoseCondition.whileActiveOnce(
                RobotCommands.PrepareLowSample()
//                MMRobot.getInstance().mmSystems.elevator.moveToPose(80).alongWith(
//                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoreSampleLow))

        );
//        setPowerCondition.whileActiveOnce(
//                new InstantCommand(() -> MMRobot.getInstance().mmSystems.elevator.setPower(0.2))
//        );
//        gotoPoseCondition2.whileActiveOnce(
//                MMRobot.getInstance().mmSystems.elevator.moveToPose(140).alongWith(
//                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoreSampleHigh)
//                )        );
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
