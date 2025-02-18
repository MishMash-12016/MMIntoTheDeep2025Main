package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

import com.qualcomm.hardware.limelightvision.Limelight3A;

//@TeleOp(name = "LimeLightTeleOp", group = "Sensor")
public class LimeLightTeleOp extends MMOpMode {
    private Limelight3A limelight;


    public LimeLightTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {
        MMRobot.getInstance().mmSystems.initRobotSystems();
        MMRobot.getInstance().mmSystems.initDriveTrain();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(1);

        limelight.pipelineSwitch(0);

        limelight.start();

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(
                        limelightGetter.getAlignToSample(limelight),
                        limelightGetter.getOpenLinearToSample(limelight)
                )
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(0),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(0)
                )
        );
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();

        telemetry.update();
    }
}
