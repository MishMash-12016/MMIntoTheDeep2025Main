package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class SampleTestTeleOp extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;

    public SampleTestTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        mmSystems = robotInstance.mmSystems;

        robotInstance.mmSystems.initRobotSystems();
        robotInstance.mmSystems.initDriveTrain();

        new Trigger(() -> mmSystems.gamepadEx1.getTrigger(
                GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
                .whileActiveOnce(IntakeSampleCommand.prepareSampleIntake(
                        () -> mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                        () -> mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).get()));


        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                IntakeSampleCommand.SampleIntake()
        );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                IntakeSampleCommand.sampleEject()
        );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                ScoringSampleCommand.PrepareScoreHigh()
        );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                ScoringSampleCommand.ScoreSample()
        );
    }
    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
       mmSystems.elevator.updateToDashboard();

    }
}
