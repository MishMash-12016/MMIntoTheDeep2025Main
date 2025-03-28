package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class BlankTeleOp extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;

    Double strafeOffset = 0.0;
    Double angle = 0.0;
    Double distance = 0.0;


    public BlankTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {
        robotInstance = MMRobot.getInstance();
        mmSystems = robotInstance.mmSystems;
        robotInstance.mmSystems.initRobotSystemsTeleOp();

        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> MMRobot.getInstance().mmSystems.vision.trackYellow()
        );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                () -> MMRobot.getInstance().mmSystems.vision.trackRed()
        );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.trackYellow()),
                        new InstantCommand(() -> strafeOffset =MMRobot.getInstance().mmSystems.vision.getStrafeOffset() ),
                        new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.trackRed()),
                        new InstantCommand(() ->distance =MMRobot.getInstance().mmSystems.vision.getDistance() ),
                        new InstantCommand(() ->angle =MMRobot.getInstance().mmSystems.vision.getTurnServoDegree() )
                        )
        );
    }

    @Override
    public void run() {
        super.run();
//        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        MMRobot.getInstance().mmSystems.controlHub.pullBulkData();
        telemetry.addData("strafeOffset teleop", strafeOffset);
        telemetry.addData("distance teleop", distance);
        telemetry.addData("angle teleop", angle);
        FtcDashboard.getInstance().getTelemetry().update();
        telemetry.update();
    }
}