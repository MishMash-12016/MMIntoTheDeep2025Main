
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
        MMRobot.getInstance().mmSystems.initDriveTrain();

        //Buttons:
        Trigger elevatorLowCondition= new Trigger(
                ()-> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.X)
        );
        Trigger elevatorHighCondition = new Trigger(
                () -> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.B)
        );
        Trigger intakeDoneCondition= new Trigger(
                ()-> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.A)
        );
        Trigger scoreSampleCondition= new Trigger(
                ()-> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.Y)
        );

        //Trigger
        Trigger rightTrigger= new Trigger(
                ()-> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.05
        );
        Trigger restYawCondition= new Trigger(
                ()-> MMRobot.getInstance().mmSystems.gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        );


        rightTrigger.whileActiveOnce(
                RobotCommands.IntakeCommand(()-> gamepad1.right_trigger)
        );
        //Buttons:
        scoreSampleCondition.whenActive(RobotCommands.ScoreSample()); //y
        elevatorHighCondition.whenActive(RobotCommands.PrepareHighSample()); //b
        elevatorLowCondition.whenActive(RobotCommands.PrepareLowSample()); //x
        intakeDoneCondition.whenActive(RobotCommands.IntakeDoneCommand()); //a
        restYawCondition.whenActive(()->MMRobot.getInstance().mmSystems.imu.resetYaw());



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
