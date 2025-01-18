package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimansCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSpecimanCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.DriveTrain.Commands.ResetFieldOrientedCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class ManualDrive extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;

    public ManualDrive() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        mmSystems = robotInstance.mmSystems;

        robotInstance.mmSystems.initRobotSystems();
        robotInstance.mmSystems.initDriveTrain();

        new Trigger(() -> mmSystems.gamepadEx1.getTrigger( //intake+intake rotater
                GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
                .whileActiveOnce(IntakeSampleCommand.prepareSampleIntake(
                        () -> mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));

        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                mmSystems.intakeEndUnitRotator.rotateByButton()
        );

        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                IntakeSampleCommand.SampleIntake() //intake - A/ X (down button)
        );

        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                ScoringSampleCommand.ScoreSample()
                //triangle
        );        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                ScoringSampleCommand.PrepareScoreLow() //O
        );        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                ScoringSampleCommand.PrepareScoreHigh()//square
        );

        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                ()-> MMRobot.getInstance().mmSystems.imu.resetYaw()
        );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                IntakeSpecimansCommand.PrepareSpecimanIntake()
        );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                IntakeSpecimansCommand.SpecimenIntake()
        );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                ScoringSpecimanCommand.SpecimanScore()
        );

    }
    @Override
    public void run() {
        super.run();

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        mmSystems.elevator.updateToDashboard();
        mmSystems.driveTrain.updateTelemetry();
        telemetry.addData("targertpose",mmSystems.elevator.targetPose);
        telemetry.addData("height",mmSystems.elevator.getHeight());
        telemetry.update();

    }
}
