package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.ActionCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.AutoSpecimensCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class ManualDrive extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;
    boolean Specimenintake = false;

    TrajectoryActionBuilder driveToScoreFirstSpecimen;

    //Second specimen
    TrajectoryActionBuilder driveToIntakeSecondSpecimen;
    TrajectoryActionBuilder driveToScoreSecondSpecimen;
    //Third specimen
    TrajectoryActionBuilder driveToIntakeThirdSpecimen;
    TrajectoryActionBuilder driveToScoreThirdSpecimen;
    //Forth specimen
    TrajectoryActionBuilder driveToIntakeForthSpecimen;
    TrajectoryActionBuilder driveToScoreForthSpecimen;


    public ManualDrive() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    private static Command score() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(0.7),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(0.15)
                ),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
        );
    }

    private SequentialCommandGroup scoreSpecimenAuto(){
        return new SequentialCommandGroup(
        new ParallelCommandGroup(
                AutoSpecimensCommand.SpecimenIntakeAuto(),
                new ActionCommand(driveToScoreFirstSpecimen.build())
        ),
                score(),

                //Second
                new ActionCommand(driveToIntakeSecondSpecimen.build()).alongWith(
                        new WaitCommand(300).andThen(
                                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake())),

                new WaitCommand(100),
                new ParallelCommandGroup(
                        AutoSpecimensCommand.SpecimenIntakeAuto(),
                        new ActionCommand(driveToScoreSecondSpecimen.build())
                ),
                score(),
                //Third

                new ActionCommand(driveToIntakeThirdSpecimen.build()).alongWith(
                        new WaitCommand(300).andThen(
                                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake())),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        AutoSpecimensCommand.SpecimenIntakeAuto(),
                        new ActionCommand(driveToScoreThirdSpecimen.build())
                ),

                //Forth
                //                robotInstance.mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.BARELY_OPEN),
                score(),

                new ActionCommand(driveToIntakeForthSpecimen.build()).alongWith(
                        new WaitCommand(300).andThen(
                                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake())),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        AutoSpecimensCommand.SpecimenIntakeAuto(),
                        new ActionCommand(driveToScoreForthSpecimen.build())
                ));

    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        mmSystems = robotInstance.mmSystems;
        Pose2d currentPose = MMRobot.getInstance().mmSystems.localizerCurrentPose;
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);


        robotInstance.mmSystems.initRobotSystems();
        robotInstance.mmSystems.initDriveTrain();
        double xPose =-28;
        TrajectoryActionBuilder driveToScoreFirstSpecimen = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(3, xPose), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2));

        //Second specimen
        TrajectoryActionBuilder driveToIntakeSecondSpecimen = driveToScoreFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(12, -40, Math.toRadians(90)), Math.atan((-40 + 59.2) / (12.0 - 45.0)), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 0.8))
                .strafeToLinearHeading(new Vector2d(45, -59.2), Math.toRadians(90));
        TrajectoryActionBuilder driveToScoreSecondSpecimen = driveToIntakeSecondSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(1, xPose), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2));

        //Third specimen
        TrajectoryActionBuilder driveToIntakeThirdSpecimen = driveToScoreSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(12, -40, Math.toRadians(90)), Math.atan((-40 + 59.2) / (12.0 - 45.0)))
                .strafeToLinearHeading(new Vector2d(45, -59.2), Math.toRadians(90));
        TrajectoryActionBuilder driveToScoreThirdSpecimen = driveToIntakeThirdSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-1, xPose), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2));

        //Forth specimen
        TrajectoryActionBuilder driveToIntakeForthSpecimen = driveToScoreSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(12, -40, Math.toRadians(90)), Math.atan((-40 + 59.2) / (12.0 - 45.0)))
                .strafeToLinearHeading(new Vector2d(45, -59.2), Math.toRadians(90));
        TrajectoryActionBuilder driveToScoreForthSpecimen = driveToIntakeForthSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-3, xPose), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2));

        TrajectoryActionBuilder park = driveToScoreForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(12, -42, Math.toRadians(90)), Math.atan((-42.0 + 57) / (12.0 - 45)))
                .strafeToLinearHeading(new Vector2d(45, -57), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2));


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
                                new InstantCommand(()-> Specimenintake= false))
                );


        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed( //specimen
                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake().alongWith(
                        new InstantCommand(()-> Specimenintake = true)
                )
        );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ConditionalCommand(
                        IntakeSpecimenCommand.SpecimenIntake(),IntakeSampleCommand.SampleIntake(),()-> Specimenintake
                )
        );
        new Trigger(() -> mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05) //slow mode
                .whenActive(
                        new ConditionalCommand(
                                ScoreSpecimenCommand.SpecimenScore(),ScoringSampleCommand.PrepareHighSample(),()-> Specimenintake
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
                .whileActiveContinuous(() -> MMRobot.getInstance().mmSystems.elevator.setPower(-1.0));
        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                ScoringSampleCommand.PrepareHighSample());


        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(0.7),
                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(0.15)
                        ),
                        new WaitCommand(200),
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                )
        );
    }

    @Override
    public void run() {
        super.run();

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        MMRobot.getInstance().mmSystems.elevator.updateToDashboard();
        mmSystems.driveTrain.updateTelemetry();
        mmSystems.driveTrain.updateTelemetry();
        telemetry.addData("switch state", mmSystems.elevator.getElevatorSwitchState());
        telemetry.addData("target pose", mmSystems.elevator.targetPose);
        telemetry.addData("ticks", mmSystems.elevator.getTicks());
        telemetry.addData("height", mmSystems.elevator.getHeight());
//        telemetry.addData("dis - ", MMRobot.getInstance().mmSystems.intakeDistSensor.getDistance());
        telemetry.addData("power", MMRobot.getInstance().mmSystems.elevator.getPower());
        telemetry.update();
        if (MMRobot.getInstance().mmSystems.gamepadEx2.gamepad.a){
            MMRobot.getInstance().mmSystems.currentMode = MMSystems.Mode.AUTOMATIC_CONTROL;
            MMRobot.getInstance().mmSystems.currentAutoActions = scoreSpecimenAuto();
        }
        else if (MMRobot.getInstance().mmSystems.gamepadEx1.gamepad.x) {
            MMRobot.getInstance().mmSystems.driveTrain.stop();
            MMRobot.getInstance().mmSystems.currentMode = MMSystems.Mode.DRIVER_CONTROL;
        }
        robotInstance.mmSystems.joystickDrive().execute();

    }
}