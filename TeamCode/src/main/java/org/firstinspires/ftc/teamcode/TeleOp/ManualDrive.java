package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class ManualDrive extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;
    private boolean SpecimenIntake;
    private boolean SpecimenScoring;

    public ManualDrive() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
        SpecimenIntake = false;
        SpecimenScoring = false;
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
                                () -> Math.pow(mmSystems.gamepadEx1.getLeftX(), 5) * 0.3,
                                () -> Math.pow(mmSystems.gamepadEx1.getLeftY(), 5) * 0.3,
                                () -> Math.pow(mmSystems.gamepadEx1.getRightX(), 1) * 0.25
                        )
                );

        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER) // sample
                .whenPressed(
                        IntakeSampleCommand.prepareSampleIntake(
                                () -> mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get(),
                                () -> mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get()
                        ).alongWith(
                                new InstantCommand(() -> SpecimenIntake = false))
                );


        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed( //specimen
                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake().alongWith(
                        new InstantCommand(() -> SpecimenIntake = true),
                        new InstantCommand(() -> SpecimenScoring = false)
                )
        );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                //TODO: separate the conditional commands to the normal way and not like this, it's soooo bad, it looks like shit and it is so fix it
                new ConditionalCommand(
                        new ConditionalCommand(ScoreSpecimenCommand.ScoreSpecimen(), new SequentialCommandGroup(
                                IntakeSpecimenCommand.SpecimenIntake(),
                                new InstantCommand(() -> SpecimenScoring = true)
                        ), () -> SpecimenScoring),
                        IntakeSampleCommand.SampleIntake(), () -> SpecimenIntake
                )
        );
        new Trigger(() -> mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
                .whenActive(
                        new ConditionalCommand(
                                ScoreSpecimenCommand.SpecimenScore(), ScoringSampleCommand.PrepareHighSample(), () -> SpecimenIntake
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
                .whenInactive(() -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0));

        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenActive(() -> MMRobot.getInstance().mmSystems.elevator.resetTicks());

        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whileActiveContinuous(() -> MMRobot.getInstance().mmSystems.elevator.setPower(-1.0));
        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                ScoringSampleCommand.PrepareHighSample());


//        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(0.7),
//                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(0.15)
//                        ),
//                        new WaitCommand(200),
//                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
//                )
//        );
    }

    @Override
    public void run() {
        super.run();

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        MMRobot.getInstance().mmSystems.elevator.updateToDashboard();
//        mmSystems.driveTrain.updateTelemetry();
//        telemetry.addData("switch state", mmSystems.elevator.getElevatorSwitchState());
//        telemetry.addData("target pose", mmSystems.elevator.targetPose);
//        telemetry.addData("ticks", mmSystems.elevator.getTicks());
//        telemetry.addData("height", mmSystems.elevator.getHeight());
////        telemetry.addData("dis - ", MMRobot.getInstance().mmSystems.intakeDistSensor.getDistance());
//        telemetry.addData("power", MMRobot.getInstance().mmSystems.elevator.getPower());
        telemetry.update();


    }
}