package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Subsystems.ShooterPID;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMTuningFFCommand;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

/**
 * obviously checking for friction in a spinwheel is pretty dumb,
 * but this is an overall example of how to use the TuningFFCommand
 */
public class TuningFFTeleOp extends MMOpMode {

    MMRobot mmRobot = MMRobot.getInstance(); //btw this is line is just to reduce code

    public TuningFFTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {

        mmRobot.mmSystems.shooterPID = new ShooterPID();

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new MMTuningFFCommand(mmRobot.mmSystems.shooterPID, 10, 0.002)
        );

    }

    @Override
    public void run() {
        super.run();
        mmRobot.mmSystems.controlHub.pullBulkData();
        telemetry.update();
    }
}
