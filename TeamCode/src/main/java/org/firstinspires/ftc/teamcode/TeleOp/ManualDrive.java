package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class ManualDrive extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;
    private boolean SpecimenIntake;
    ElapsedTime elapsedTime = new ElapsedTime();

    public ManualDrive() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
        SpecimenIntake = true;
        elapsedTime.reset();
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        mmSystems = robotInstance.mmSystems;


        robotInstance.mmSystems.initRobotSystems();
        robotInstance.mmSystems.initDriveTrain();

        //drive
        new Trigger(() -> mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05).whileActiveContinuous(
                MMRobot.getInstance().mmSystems.driveTrain.fieldOrientedDrive(
                        () -> Math.pow(mmSystems.gamepadEx1.getLeftX(), 5) * 0.3,
                        () -> Math.pow(mmSystems.gamepadEx1.getLeftY(), 5) * 0.3,
                        () -> Math.pow(mmSystems.gamepadEx1.getRightX(), 1) * 0.25
                )
        );

        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                () -> mmSystems.driveTrain.resetRotation()
        );

        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                ()->MMRobot.getInstance().mmSystems.vision.trackYellow()
        );

        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                ()->MMRobot.getInstance().mmSystems.vision.trackRed()
        );


//                 LIMELIGHT HAS RETURNED...
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                IntakeSampleCommand.limeLightIntake_TeleOp(hardwareMap).alongWith(
                        new InstantCommand(() -> SpecimenIntake = false)
                )
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                 new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SAMPLE_TRANSFER_POSE),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SAMPLE_TRANSFER_POSE)//be prepared for transfer
                 )
        );


                //prepareSampleIntake
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                IntakeSampleCommand.prepareSampleIntake(
                        () -> mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get(),
                        () -> mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get()
                ).alongWith(
                        new InstantCommand(() -> SpecimenIntake = false))
        );

        //prepareSpecimenIntake
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                IntakeSpecimenCommand.PrepareSpecimenIntakeFront().alongWith(
                        new InstantCommand(() -> SpecimenIntake = true)
                )
        );

        //sample/specimen intake
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ConditionalCommand(IntakeSpecimenCommand.IntakeFromFront(), IntakeSampleCommand.SampleIntake(), () -> SpecimenIntake)
        );

        new Trigger(() -> mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
                .whenActive(
                        new ConditionalCommand(
                                ScoreSpecimenCommand.ScoreSpecimen(), ScoringSampleCommand.PrepareHighSample(), () -> SpecimenIntake
                        )
                );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                ScoringSampleCommand.ScoreHighSample()
        );



        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
                .whileActiveContinuous(() -> MMRobot.getInstance().mmSystems.elevator.setPower(-1.0)); //left trigger

        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
                .whenInactive(() -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0));

        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
                .whileActiveContinuous(() -> MMRobot.getInstance().mmSystems.elevator.setPower(1.0)); //right trigger

        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
                .whenInactive(() -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0));

        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenActive(() -> MMRobot.getInstance().mmSystems.elevator.resetTicks());

        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whileActiveContinuous(() -> MMRobot.getInstance().mmSystems.elevator.setPower(-1.0));

        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                ScoringSampleCommand.PrepareHighSample());
    }

    @Override
    public void run() {
        super.run();
        //FOR CONFIG EXTREPULATION:
//      MMRobot.getInstance().mmSystems.linearIntake.setPositionVoid(LinearIntake.config);

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
//        MMRobot.getInstance().mmSystems.elevator.updateToDashboard();
//        mmSystems.driveTrain.updateTelemetry();
//        FtcDashboard.getInstance().getTelemetry().addData("speed X",MMSystems.localizer.getVelocityRR().linearVel.x);
//        FtcDashboard.getInstance().getTelemetry().addData("speed Y",MMSystems.localizer.getVelocityRR().linearVel.y);
        FtcDashboard.getInstance().getTelemetry().addData("speed ANG",MMSystems.localizer.getVelocityRR().angVel);
//        FtcDashboard.getInstance().getTelemetry().addData("time",elapsedTime.milliseconds());

        FtcDashboard.getInstance().getTelemetry().update();
        telemetry.addData("target pose", mmSystems.elevator.targetPose);
        telemetry.addData("ticks", mmSystems.elevator.getTicks());
        telemetry.addData("height", mmSystems.elevator.getHeight());
        telemetry.addData("power", MMRobot.getInstance().mmSystems.elevator.getPower());
        telemetry.update();
    }
}