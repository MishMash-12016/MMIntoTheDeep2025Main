package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class ManualDrive extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;
    boolean Specimenintake = true;

    public ManualDrive() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        mmSystems = robotInstance.mmSystems;


        robotInstance.mmSystems.initRobotSystems();
        robotInstance.mmSystems.initDriveTrain();

        new Trigger(() -> mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05) //slow mode
                .whileActiveContinuous(
                        MMRobot.getInstance().mmSystems.driveTrain.fieldOrientedDrive(
                                () -> Math.pow(mmSystems.gamepadEx1.getLeftX(), 3) * 0.5,
                                () -> Math.pow(mmSystems.gamepadEx1.getLeftY(), 3) * 0.5,
                                () -> Math.pow(mmSystems.gamepadEx1.getRightX(), 3) * 0.25)

                );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER) // sample
                .whenPressed(
                        IntakeSampleCommand.prepareSampleIntake(() -> mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get(),() -> mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get()).alongWith(
                                new InstantCommand(()-> Specimenintake= false)));


        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed( //specimen
                IntakeSpecimansCommand.PrepareSpecimanIntake().alongWith(
                        new InstantCommand(()-> Specimenintake= true)
                )
        );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ConditionalCommand(
                        IntakeSpecimansCommand.SpecimenIntake(),IntakeSampleCommand.SampleIntake(),()-> Specimenintake
                )
        );
        new Trigger(() -> mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05) //slow mode
                .whenActive(
                        new ConditionalCommand(
                                ScoringSpecimanCommand.SpecimanScore(),ScoringSampleCommand.PrepareHighSample(),()-> Specimenintake
                        )
                );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                ScoringSampleCommand.ScoreHighSample()
        );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                () -> mmSystems.driveTrain.resetRotation()
        );

        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
                .whileActiveContinuous(() -> MMRobot.getInstance().mmSystems.elevator.setPower(-0.6)); //left trigger
        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
                .whenInactive(() -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0));
        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
                .whileActiveContinuous(() -> MMRobot.getInstance().mmSystems.elevator.setPower(0.6)); //right trigger
        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
                .whenInactive(()-> MMRobot.getInstance().mmSystems.elevator.setPower(0.0));
        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenActive(() -> MMRobot.getInstance().mmSystems.elevator.resetTicks());
        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenActive(MMRobot.getInstance().mmSystems.elevator.moveToPose(0.05));
        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                 ScoringSampleCommand.PrepareHighSample());




    }

    @Override
    public void run() {
        super.run();

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        MMRobot.getInstance().mmSystems.elevator.updateToDashboard();
        mmSystems.driveTrain.updateTelemetry();
        telemetry.addData("targertpose", mmSystems.elevator.targetPose);
        telemetry.addData("ticks - ", mmSystems.elevator.getTicks());
        telemetry.addData("height", mmSystems.elevator.getHeight());
        telemetry.update();

    }
}
