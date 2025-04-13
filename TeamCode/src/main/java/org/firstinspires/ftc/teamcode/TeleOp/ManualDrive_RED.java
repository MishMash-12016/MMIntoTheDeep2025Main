package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Red_Right_6;
import org.firstinspires.ftc.teamcode.CommandGroup.ClimbingCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.RotateIntakeServoByRobotAngle;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class ManualDrive_RED extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;
    private boolean SpecimenIntake;
    private double elbowOffset;
    ElapsedTime elapsedTime = new ElapsedTime();


    public ManualDrive_RED() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
        SpecimenIntake = true;
        elapsedTime.reset();
    }

    @Override
    public void onInit() {
        elbowOffset = 0;
        robotInstance = MMRobot.getInstance();
        mmSystems = robotInstance.mmSystems;

        robotInstance.mmSystems.initRobotSystemsDontMove(this);
        robotInstance.mmSystems.initDriveTrain();
        robotInstance.mmSystems.teleop();
        robotInstance.mmSystems.vision.trackYellow();
//        robotInstance.mmSystems.driveTrain.pose = MMSystems.AutoPose;
//        robotInstance.mmSystems.currentPose = MMSystems.AutoPose;

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



//                 LIMELIGHT HAS RETURNED...
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
//                        new InstantCommand(()->MMRobot.getInstance().mmSystems.auto(), MMRobot.getInstance().mmSystems.driveTrain),
                        IntakeSampleCommand.limeLightIntake_TeleOp().alongWith(
                                new InstantCommand(() -> SpecimenIntake = false)
                        )
        );
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                IntakeSampleCommand.doLimelightIntake_Auto(IntakeEndUnitRotator.defaultPose).alongWith(
                        new InstantCommand(() -> SpecimenIntake = false)
                )
        );

                //prepareSampleIntake
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                IntakeSampleCommand.prepareSampleIntake(
                                () -> mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get(),
                                () -> mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get()
                ).alongWith(
                        new InstantCommand(() -> SpecimenIntake = false)
                )
        );

        //prepareSpecimenIntake
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE),
                                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE),
                                robotInstance.mmSystems.intakEndUnit.openIntakeClaw()

                        ),
                        IntakeSpecimenCommand.PrepareSpecimenIntakeFront().alongWith(
                                new InstantCommand(() -> SpecimenIntake = true)
                        ),
                        () -> (robotInstance.mmSystems.linearIntake.pose == LinearIntake.LinearIntakeState.MAX_OPENING.position)
                )
        );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new RotateIntakeServoByRobotAngle(225,140, 0,0.15)
        );
        //sample/specimen intake
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ConditionalCommand(IntakeSpecimenCommand.SpecimenIntake(), IntakeSampleCommand.SampleIntake(), () -> SpecimenIntake)
        );

        new Trigger(() -> mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05).whenActive(
                ScoringSampleCommand.PrepareHighSample()
        );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                ScoringSampleCommand.ScoreHighSample()
        );

        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(
                new InstantCommand(()->MMRobot.getInstance().mmSystems.vision.trackRed())
        );

        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(
                new InstantCommand(()->MMRobot.getInstance().mmSystems.vision.trackYellow())
        );


//        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
//                .whileActiveContinuous(() -> MMRobot.getInstance().mmSystems.elevator.setPower(-1.0)); //left trigger
//
//        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
//                .whenInactive(() -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0));
//
//        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
//                .whileActiveContinuous(() -> MMRobot.getInstance().mmSystems.elevator.setPower(1.0)); //right trigger
//
//        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
//                .whenInactive(() -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0));

//        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
//                .whileActiveContinuous(MMRobot.getInstance().mmSystems.climber.setPower(-1.0)); //left trigger
//
//        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
//                .whileActiveContinuous(MMRobot.getInstance().mmSystems.climber.setPower(1.0)); //right trigger

        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                () -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0)
        );

        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new SequentialCommandGroup(
                        ScoringSampleCommand.PrepareHighSampleEran()
                )
        );

        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                ()-> MMRobot.getInstance().mmSystems.vision.switchToPython()
        );

//        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> {
//                            elbowOffset -= 0.035;
//                            ScoringEndUnitElbow.prepareSampleScorePose += elbowOffset;
//                            ScoringEndUnitElbow.ElbowScoreSamplePose += elbowOffset;
//                            ScoringEndUnitElbow.ElbowInitPose += elbowOffset;
//                            ScoringEndUnitElbow.ElbowTransferSamplePose += elbowOffset;
//                            ScoringEndUnitElbow.ElbowPrepareSampleTransferPose += elbowOffset;
//                            ScoringEndUnitElbow.ElbowAfterScoreSpecimenPose += elbowOffset;
//                            ScoringEndUnitElbow.ElbowIntakeFromFrontPose += elbowOffset;
//                            ScoringEndUnitElbow.scoringElbowScoreFromFrontPose += elbowOffset;
//                            ScoringEndUnitElbow.ElbowAfterScoreFromFront += elbowOffset;
//                            ScoringEndUnitElbow.ElbowPark += elbowOffset;
//                            ScoringEndUnitElbow.est += elbowOffset;
//                        }),
//                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.est)
//                )
//        );
//
//
//        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> {
//                            elbowOffset += 0.035;
//                            ScoringEndUnitElbow.prepareSampleScorePose += elbowOffset;
//                            ScoringEndUnitElbow.ElbowScoreSamplePose += elbowOffset;
//                            ScoringEndUnitElbow.ElbowInitPose += elbowOffset;
//                            ScoringEndUnitElbow.ElbowTransferSamplePose += elbowOffset;
//                            ScoringEndUnitElbow.ElbowPrepareSampleTransferPose += elbowOffset;
//                            ScoringEndUnitElbow.ElbowAfterScoreSpecimenPose += elbowOffset;
//                            ScoringEndUnitElbow.ElbowIntakeFromFrontPose += elbowOffset;
//                            ScoringEndUnitElbow.scoringElbowScoreFromFrontPose += elbowOffset;
//                            ScoringEndUnitElbow.ElbowAfterScoreFromFront += elbowOffset;
//                            ScoringEndUnitElbow.ElbowPark += elbowOffset;
//                            ScoringEndUnitElbow.est += elbowOffset;
//                        }),
//                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.est)
//                )
//        );

        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                ClimbingCommand.PrepareClimbToThird()
        );

        mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                ClimbingCommand.ClimbToThird()
        );


        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
                .whileActiveContinuous(
                        new RunCommand(
                                () -> MMRobot.getInstance().mmSystems.elevator.setPower(mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)),
                                MMRobot.getInstance().mmSystems.elevator
                        )
                ).whenInactive(
                        new RunCommand(
                                () -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0),
                                MMRobot.getInstance().mmSystems.elevator
                        )
                );

        new Trigger(() -> mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
                .whileActiveContinuous(
                        new RunCommand(
                                () -> MMRobot.getInstance().mmSystems.elevator.setPower(-mmSystems.gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)),
                                MMRobot.getInstance().mmSystems.elevator
                        )
                ).whenInactive(
                        new RunCommand(
                                () -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0),
                                MMRobot.getInstance().mmSystems.elevator
                        )
                );

    }

    @Override
    public void run() {
        super.run();

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        FtcDashboard.getInstance().getTelemetry().update();
        telemetry.update();
    }


    public static double getAng() {
        double ang = Math.toDegrees(MMSystems.AutoPose.heading.toDouble());
        ang = ang < 0 ? ang + 360 : ang;
        return ang;
    }
}