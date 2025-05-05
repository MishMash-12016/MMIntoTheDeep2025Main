package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.roadRunnewr.sampleGoToScore;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.utils.FixedSequentialCommandGroup;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

//TODO:rot
@Autonomous
public class AutoSample7_Red extends MMOpMode {
    static MMRobot robotInstance;
    public static final Pose2d scorePose = new Pose2d(-58.5, -54.5, Math.toRadians(240));
    final Pose2d intakePose = new Pose2d(-24, -9, Math.toRadians(180));

    boolean flag = false;

    public AutoSample7_Red() {
        super(OpModeType.Competition.AUTO);
    }

    @Override
    public void onInit() {
        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems(this);

        MMRobot.getInstance().mmSystems.vision.trackRedAndYellow();

        Pose2d currentPose = new Pose2d(-38.23, -64.5, Math.toRadians(PinpointDrive._autoStartAngle = 180));
        PinpointDrive._autoStartAngle += 90;
        MMSystems.localizer = null;
        MMRobot.getInstance().mmSystems.initLocalize(currentPose);
        robotInstance.mmSystems.initDriveTrain(currentPose);
        PinpointDrive drive = MMRobot.getInstance().mmSystems.driveTrain;
        drive.pinpoint.setPosition(currentPose);
        drive.pinpoint.update();

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);

        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(-62, -54), Math.toRadians(244));

        TrajectoryActionBuilder driveToIntakeFirst = driveToScorePreloadSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(244 + 180))
                .splineToConstantHeading(new Vector2d(-59, -46.2), Math.toRadians(244 - 180),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 0.7),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.4, MecanumDrive.PARAMS.maxProfileAccel * 0.7));

        TrajectoryActionBuilder driveToScoreFirst = driveToIntakeFirst.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-62, -52.5), Math.toRadians(263));

        TrajectoryActionBuilder driveToIntakeSecondSample = driveToScoreFirst.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-58.7, -48.5), Math.toRadians(268),
                        null, new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.45, MecanumDrive.PARAMS.maxProfileAccel * 0.6));

        TrajectoryActionBuilder driveToScoreSecondSample = driveToIntakeSecondSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-61.6, -51.2), Math.toRadians(250),
                        null, new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.7, MecanumDrive.PARAMS.maxProfileAccel));

        TrajectoryActionBuilder driveToIntakeThird = driveToScoreSecondSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-57.7, -45.9), Math.toRadians(299),
                        null, new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.7, MecanumDrive.PARAMS.maxProfileAccel));

        TrajectoryActionBuilder driveToScoreThird = driveToIntakeThird.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-61.2, -51.4), Math.toRadians(245),
                        null, new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.7, MecanumDrive.PARAMS.maxProfileAccel));

        TrajectoryActionBuilder driveToIntakeForth = driveToScoreThird.endTrajectory().fresh()
                .setTangent(Math.toRadians(70))
                .splineToSplineHeading(intakePose, Math.toRadians(25),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.3, MecanumDrive.PARAMS.maxProfileAccel * 1.2));

        TrajectoryActionBuilder driveToScoreForth = driveToIntakeForth.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(-38, -20), Math.toRadians(240))
                .splineTo(scorePose.component1(), Math.toRadians(230),
                        null,
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.3, MecanumDrive.PARAMS.maxProfileAccel));
//                .setTangent(Math.toRadians(180))
//                .splineTo(scorePose.component1(), Math.toRadians(240),
//                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel*0.8),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel*1.2));

        TrajectoryActionBuilder driveToIntakeFifth = driveToScoreForth.endTrajectory().fresh()
                .setTangent(Math.toRadians(70))
                .splineToSplineHeading(intakePose, Math.toRadians(25),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.3, MecanumDrive.PARAMS.maxProfileAccel * 1.2));

        TrajectoryActionBuilder driveToScoreFifth = driveToIntakeFifth.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(-38, -20), Math.toRadians(240))
                .splineTo(scorePose.component1(), Math.toRadians(230),
                        null,
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.3, MecanumDrive.PARAMS.maxProfileAccel));

        TrajectoryActionBuilder driveToIntakeSixth = driveToScoreFifth.endTrajectory().fresh()
                .setTangent(Math.toRadians(70))
                .splineToSplineHeading(intakePose, Math.toRadians(25),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.3, MecanumDrive.PARAMS.maxProfileAccel * 1.2));

        TrajectoryActionBuilder driveToScoreSixth = driveToIntakeSixth.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(-38, -20), Math.toRadians(240))
                .splineTo(scorePose.component1(), Math.toRadians(230),
                        null,
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.3, MecanumDrive.PARAMS.maxProfileAccel));

        TrajectoryActionBuilder driveToPark = driveToScoreSixth.endTrajectory().fresh()
                .setTangent(Math.toRadians(70))
                .splineToSplineHeading(new Pose2d(-16, -9, Math.toRadians(180)), Math.toRadians(25),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.7, MecanumDrive.PARAMS.maxProfileAccel * 1.2));

        new SequentialCommandGroup(
                new InstantCommand(() -> drive.pinpoint.setPosition(currentPose)),

                new ActionCommand(driveToScorePreloadSample.build()).alongWith(
                        new SequentialCommandGroup(
                                scorePreLoadSample().interruptOn(
                                        () -> MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR().position.x < -58.5
                                ),
                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                prepareSampleIntakeWithoutButtonAndScoring().alongWith(
                                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.36)
                                )
                        )
                ),

                new ActionCommand(driveToIntakeFirst.build()).alongWith(
                        new SequentialCommandGroup(
                                new WaitCommand(400),
                                new ParallelCommandGroup(
                                        MMRobot.getInstance().mmSystems.elevator.ElevatorGetToZeroSensor(),
                                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SAMPLE_POSE),
                                        new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.elevator.getHeight() < 10).andThen(
                                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE)
                                        )
                                )
                        )
                ),


                new SequentialCommandGroup(
                        SampleIntake(),
                        new ActionCommand(driveToScoreFirst.build()).alongWith(
                                new WaitCommand(400).andThen(
                                        prepareHighWithOutIntakeSecond()
                                )
                        )

                ),

                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                ScoreHighSampleWithoutIntake(),
                                new SequentialCommandGroup(
                                        new ActionCommand(driveToIntakeSecondSample.build()),
                                        SampleIntake()
                                )
                        ),
                        new ActionCommand(driveToScoreSecondSample.build()).alongWith(
                                new WaitCommand(400).andThen(
                                        prepareHighWithOutIntake()
                                )
                        )

                ),

                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                ScoreHighSampleWithoutIntake(),
                                new SequentialCommandGroup(
                                        new ActionCommand(driveToIntakeThird.build()).alongWith(
                                                prepareSampleIntakeWithoutButtonAndScoring().alongWith(
                                                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.3)
                                                )
                                        ),
                                        SampleIntake()
                                )
                        ),
                        new ActionCommand(driveToScoreThird.build()).alongWith(
                                new WaitCommand(400).andThen(
                                        prepareHighWithOutIntake()
                                )
                        )
                ),

                new ParallelCommandGroup(
                        new ActionCommand(driveToIntakeForth.build()),
                        ScoringSampleCommand.ScoreHighSample(),
                        new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.switchToDetector())
                ),
                new WaitCommand(500),
                IntakeSampleCommand.limeLightIntake_TeleOpButAuto_First().withTimeout(3000),

                new sampleGoToScore().alongWith(
                        new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.switchToDetector()),
                        ScoringSampleCommand.PrepareHighSample_Auto()
                ),

                new ParallelCommandGroup(
                        new ActionCommand(driveToIntakeFifth.build()),
                        ScoringSampleCommand.ScoreHighSample()
                ),
                new WaitCommand(500),
                IntakeSampleCommand.limeLightIntake_TeleOpButAuto().withTimeout(3000),


                new sampleGoToScore().alongWith(
                        ScoringSampleCommand.PrepareHighSample_Auto()
                ),

                new ActionCommand(driveToPark.build()).alongWith(
                        //basically scoring
                        new SequentialCommandGroup(
                                ScoringSampleCommand.ScoreHighSample(),
                                new WaitCommand(200),
                                new ParallelCommandGroup(
                                        MMRobot.getInstance().mmSystems.elevator.ElevatorGetToZeroSensor(),
                                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PARK),
                                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PARK),
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw()
                                )
                        )
                )
        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        MMSystems.AutoPose = MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR();

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.addData("flag", flag);
        telemetry.addData("ele", MMRobot.getInstance().mmSystems.elevator.getElevatorSwitchState());
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }

    public Command scorePreLoadSample() {
        return new FixedSequentialCommandGroup(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORE_ARM_SCORE_SAMPLE),
                        MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET).withTimeout(2500),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_SCORE),
                                new WaitUntilCommand(() ->
                                        MMRobot.getInstance().mmSystems.elevator.getHeight() < Elevator.ElevatorState.HIGH_BASKET.position.get() - 7).andThen(
                                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE)
                                )
                        )
                )
        );
    }

    public static Command ScoreHighSampleWithoutIntake() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.elevator.ElevatorGetToZeroSensor(),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SAMPLE_POSE),
                        new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.elevator.getHeight() < 10).andThen(
                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE)
                        )
                )
        );
    }

    public static Command prepareSampleIntakeWithoutButtonAndScoring() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(0.51),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                new WaitCommand(70).andThen(
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                ));
    }

    public static Command SampleIntake() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SAMPLE_INTAKE_POSE),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SAMPLE_TRANSFER_POSE),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INIT_POSE),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE)
        );
    }

    public static Command prepareHighWithOutIntake() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SAMPLE_POSE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SAMPLE_TRANSFER_POSE)
                ),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
//                new WaitCommand(100),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET).withTimeout(2500),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORE_ARM_SCORE_SAMPLE),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_SCORE),
                                new WaitUntilCommand(() ->
                                        MMRobot.getInstance().mmSystems.elevator.getHeight() < Elevator.ElevatorState.HIGH_BASKET.position.get() - 7).andThen(
                                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE)
                                )
                        )
                )
        );
    }


    public static Command prepareHighWithOutIntakeSecond() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SAMPLE_POSE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SAMPLE_TRANSFER_POSE)
                ),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
//                new WaitCommand(100),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET).withTimeout(2500),
                        new SequentialCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORE_ARM_SCORE_SAMPLE),
                                new WaitCommand(300),
                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_SCORE),
                                new ParallelCommandGroup(
                                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0.51),
                                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                                ),
                                new WaitUntilCommand(() ->
                                        MMRobot.getInstance().mmSystems.elevator.getHeight() < Elevator.ElevatorState.HIGH_BASKET.position.get() - 7).andThen(
                                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE),
                                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORING_ARM_SCORE_POSE)
                                )
                        )
                )
        );
    }
}
