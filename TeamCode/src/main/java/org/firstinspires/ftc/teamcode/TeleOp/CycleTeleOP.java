package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.RobotCommands;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
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
        MMRobot.getInstance().mmSystems.initDriveTrain();

        //Buttons:
        //Gamepad 1:
        Trigger intakeCondition = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05);
        Trigger intakeDoneCondition = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.A));
        Trigger changeIntakeRotator = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.B));
        Trigger scoreSampleCondition = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.Y));
        Trigger restYawCondition = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER));

        //Gamepad 2:
        Trigger prepareHighSampleCondition = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.DPAD_UP));
        Trigger prepareLowSampleCondition = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.DPAD_DOWN));
        Trigger foldSystemCondition = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.B));
        Trigger ejectCondition = new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx2.getButton(GamepadKeys.Button.Y));


        //Buttons:
        //gamepad 1: intake,intake done,rotate,score,drive,rest imu
        intakeCondition.whileActiveOnce(RobotCommands.IntakeCommand(() -> gamepad1.right_trigger) //right trigger
        );
        intakeDoneCondition.whenActive(RobotCommands.IntakeDoneCommand()); //a
        changeIntakeRotator.whenActive(MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(
                        IntakeEndUnitRotator.rotateangle)); //B
        scoreSampleCondition.whenActive(RobotCommands.ScoreSample()); //y
        restYawCondition.whenActive(() -> MMRobot.getInstance().mmSystems.imu.resetYaw()); //right bumper

        //gamepad 2: prepare high,prepare low,fold,eject
        prepareHighSampleCondition.whenActive(RobotCommands.PrepareHighSample()); // dpad up
        prepareLowSampleCondition.whenActive(RobotCommands.PrepareLowSample()); //dpad down
        foldSystemCondition.whenActive(RobotCommands.FoldSystems()); // B
        ejectCondition.whenActive(RobotCommands.EjectSampleCommand()); // Y


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
