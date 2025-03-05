package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "LimeLightTeleOp", group = "Sensor")
public class LimeLightTeleOp extends MMOpMode {


    public LimeLightTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {
        MMRobot.getInstance().mmSystems.initRobotSystems();
        MMRobot.getInstance().mmSystems.initDriveTrain();

//        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new SequentialCommandGroup(
//                        limelightGetter.getOpenLinearToSample(limelight),
//                        limelightGetter.getRotateToSample(limelight)
//                )
//        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new SequentialCommandGroup(
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                        limelightGetter.strafeToSample(),
                        limelightGetter.getRotateToSample(),
                        limelightGetter.getOpenLinearToSample(),
                        new WaitCommand(500),
                        MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                        new WaitCommand(500),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(0.185),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0)
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(1)
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INTAKE_SAMPLE_POSE),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(0.15)


                )
        );

        new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
                .whileActiveContinuous(
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(
                                () -> MMRobot.getInstance().mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                        ).alongWith(MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE))); //right trigger
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.joystickDrive().initialize();
        MMRobot.getInstance().mmSystems.joystickDrive().execute();
        MMRobot.getInstance().mmSystems.controlHub.pullBulkData();
        MMRobot.getInstance().mmSystems.telemetry.addData("trigger", MMRobot.getInstance().mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        telemetry.update();
    }
}
