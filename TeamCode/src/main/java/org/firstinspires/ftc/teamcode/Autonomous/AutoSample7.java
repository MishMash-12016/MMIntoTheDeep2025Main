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
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringArm_SpeedControll;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

import java.util.function.BooleanSupplier;

//TODO:rot
@Autonomous
public class AutoSample7 extends MMOpMode {
    static MMRobot robotInstance;
    final Pose2d scorePose = new Pose2d(-59, -50, Math.toRadians(240));
    final Pose2d intakePose = new Pose2d(-26, -12, Math.toRadians(180));

    boolean flag = false;

    public AutoSample7() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {
        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        MMRobot.getInstance().mmSystems.vision.trackYellow();
        MMRobot.getInstance().mmSystems.vision.switchToDetector();

        Pose2d currentPose = new Pose2d(-38.23, -65, Math.toRadians(180));
        robotInstance.mmSystems.initDriveTrain(currentPose);
        PinpointDrive drive = MMRobot.getInstance().mmSystems.driveTrain;

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);

        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(-58.7, -52.3), Math.toRadians(246));

        TrajectoryActionBuilder driveToIntakeFirst = driveToScorePreloadSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-59.4, -46.2), Math.toRadians(246));

        TrajectoryActionBuilder driveToScoreFirst = driveToIntakeFirst.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-59.2, -51.8), Math.toRadians(257));

        TrajectoryActionBuilder driveToIntakeSecondSample = driveToScoreFirst.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-59.9, -48.5), Math.toRadians(268),
                        null, new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.45, MecanumDrive.PARAMS.maxProfileAccel * 0.6));

        TrajectoryActionBuilder driveToScoreSecondSample = driveToIntakeSecondSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-61.4, -51), Math.toRadians(250),
                        null, new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.7, MecanumDrive.PARAMS.maxProfileAccel));

        TrajectoryActionBuilder driveToIntakeThird = driveToScoreSecondSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-57.7, -46.7), Math.toRadians(298),
                        null, new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.7, MecanumDrive.PARAMS.maxProfileAccel));

        TrajectoryActionBuilder driveToScoreThird = driveToIntakeThird.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-61.2, -51.4), Math.toRadians(245),
                        null, new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.7, MecanumDrive.PARAMS.maxProfileAccel));

        TrajectoryActionBuilder driveToIntakeForth = driveToScoreThird.endTrajectory().fresh()
                .setTangent(Math.toRadians(62))
                .splineToSplineHeading(intakePose, Math.toRadians(15)
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));

        TrajectoryActionBuilder driveToScoreForth = driveToIntakeForth.endTrajectory().fresh()
                .setTangent(Math.toRadians(200))
                .splineToSplineHeading(scorePose, Math.toRadians(240)
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));

        TrajectoryActionBuilder driveToIntakeFifth = driveToScoreForth.endTrajectory().fresh()
                .setTangent(Math.toRadians(62))
                .splineToSplineHeading(intakePose, Math.toRadians(15)
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));

        TrajectoryActionBuilder driveToScoreFifth = driveToIntakeFifth.endTrajectory().fresh()
                .setTangent(Math.toRadians(200))
                .splineToSplineHeading(scorePose, Math.toRadians(240)
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));

        TrajectoryActionBuilder driveToIntakeSixth = driveToScoreFifth.endTrajectory().fresh()
                .setTangent(Math.toRadians(62))
                .splineToSplineHeading(intakePose, Math.toRadians(15)
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));

        TrajectoryActionBuilder driveToScoreSixth = driveToIntakeSixth.endTrajectory().fresh()
                .setTangent(Math.toRadians(200))
                .splineToSplineHeading(scorePose, Math.toRadians(240)
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));

        TrajectoryActionBuilder driveToPark = driveToScoreSixth.endTrajectory().fresh()
                .setTangent(Math.toRadians(62))
                .splineToSplineHeading(intakePose, Math.toRadians(15)
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));

        new SequentialCommandGroup(
                new InstantCommand(),

                new ActionCommand(driveToScorePreloadSample.build()).alongWith(
                        scorePreLoadSample(),
                        new ScoringArm_SpeedControll(500, ScoringArm.ScoringArmState.SCORING_ARM_SCORE_POSE.position.get())
                ),
                new WaitCommand(100),

                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                ScoreHighSampleWithoutIntake(),
                                new SequentialCommandGroup(
                                        new ActionCommand(driveToIntakeFirst.build()).alongWith(
                                                prepareSampleIntakeWithoutButtonAndScoring()
                                        ),
                                        SampleIntake()
                                )
                        ),
                        new ActionCommand(driveToScoreFirst.build()).alongWith(
                                new WaitCommand(200).andThen(
                                        prepareHighWithOutIntake()
                                )
                        )

                ),

                new WaitCommand(200),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                ScoreHighSampleWithoutIntake(),
                                new SequentialCommandGroup(
                                        new ActionCommand(driveToIntakeSecondSample.build()).alongWith(
                                                prepareSampleIntakeWithoutButtonAndScoring()
                                        ),
                                        SampleIntake()
                                )
                        ),
                        new ActionCommand(driveToScoreSecondSample.build()).alongWith(
                                new WaitCommand(200).andThen(
                                        prepareHighWithOutIntake()
                                )
                        )

                ),

                new WaitCommand(200),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                ScoreHighSampleWithoutIntake(),
                                new SequentialCommandGroup(
                                        new ActionCommand(driveToIntakeThird.build()).alongWith(
                                                prepareSampleIntakeWithoutButtonAndScoring()
                                        ),
                                        SampleIntake()
                                )
                        ),
                        new ActionCommand(driveToScoreThird.build()).alongWith(
                                new WaitCommand(200).andThen(
                                        prepareHighWithOutIntake()
                                )
                        )
                ),

                new ParallelCommandGroup(
                        new ActionCommand(driveToIntakeForth.build()),
                        ScoringSampleCommand.ScoreHighSample(),
                        new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.switchToDetector())
                ),

                IntakeSampleCommand.limeLightIntake_Auto(),

                new ActionCommand(driveToScoreForth.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample_Auto()
                ),

                new ParallelCommandGroup(
                        new ActionCommand(driveToIntakeFifth.build()),
                        ScoringSampleCommand.ScoreHighSample(),
                        new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.switchToDetector())
                ),

                IntakeSampleCommand.limeLightIntake_Auto(),

                new ActionCommand(driveToScoreFifth.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample_Auto()
                ),
                new ParallelCommandGroup(
                        new ActionCommand(driveToIntakeSixth.build()),
                        ScoringSampleCommand.ScoreHighSample(),
                        new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.switchToDetector())
                ),

                IntakeSampleCommand.limeLightIntake_Auto(),

                new ActionCommand(driveToScoreSixth.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample_Auto()
                ),
                new ActionCommand(driveToPark.build()).alongWith(
                        //basically scoring
                        new SequentialCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                new WaitCommand(200),
                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORE_ARM_SCORE_SAMPLE),
                                new WaitCommand(200),
                                new ParallelCommandGroup(
                                        MMRobot.getInstance().mmSystems.elevator.ElevatorGetToZeroSensor(),
                                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE),
                                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE)
                                )
                        )
                )
        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.addData("flag", flag);
        telemetry.addData("ele", MMRobot.getInstance().mmSystems.elevator.getElevatorSwitchState());
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }

    public Command scorePreLoadSample() {
        return new SequentialCommandGroup(
                new WaitCommand(200),
                new RunCommand(() -> MMRobot.getInstance().mmSystems.elevator.setPower(0.8), MMRobot.getInstance().mmSystems.elevator)
                        .interruptOn(() -> MMRobot.getInstance().mmSystems.elevator.getHeight() > 43),
                new InstantCommand(() -> MMRobot.getInstance().mmSystems.elevator.setPower(0.0), MMRobot.getInstance().mmSystems.elevator),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE)
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
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE)
        );
    }

    public static Command prepareHighWithOutIntake() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SAMPLE_POSE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SAMPLE_TRANSFER_POSE)
                ),
                new WaitCommand(150),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                new WaitCommand(100),

                new WaitCommand(100),
                new ParallelCommandGroup(
                        new ScoringArm_SpeedControll(500, ScoringArm.ScoringArmState.SCORE_ARM_SCORE_SAMPLE.position.get()),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_SCORE),
                        MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET)
                ),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE)
        );
    }
}
