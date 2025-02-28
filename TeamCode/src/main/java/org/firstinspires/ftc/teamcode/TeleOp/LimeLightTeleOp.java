package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "LimeLightTeleOp", group = "Sensor")
public class LimeLightTeleOp extends MMOpMode {
    private Limelight3A limelight;
    PinpointDrive drive;


    public LimeLightTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {
        MMRobot.getInstance().mmSystems.initRobotSystems();
        MMRobot.getInstance().mmSystems.initDriveTrain();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(1);
        Pose2d currentPose = new Pose2d(0, 0, Math.toRadians(0));

        drive = new PinpointDrive(hardwareMap, currentPose);

        limelight.start();

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(
                        limelightGetter.getOpenLinearToSample(limelight),
                        limelightGetter.getRotateToSample(limelight)
                )
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new SequentialCommandGroup(
                        limelightGetter.strafeToSample(limelight, drive)
                )
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(0.15)
                        )
        );
    }

    @Override
    public void run() {
        super.run();
        drive.updatePoseEstimate();
//        MMRobot.getInstance().mmSystems.joystickDrive().execute();
        MMRobot.getInstance().mmSystems.controlHub.pullBulkData();

        telemetry.update();
    }
}
