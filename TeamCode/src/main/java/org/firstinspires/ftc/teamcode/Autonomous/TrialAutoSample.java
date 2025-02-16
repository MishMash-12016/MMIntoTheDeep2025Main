package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.LazyActionCommand;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class TrialAutoSample extends MMOpMode {

    public static double LIMELIGHT_INFO = -1;
    public static TrajectoryActionBuilder LIMELIGHT_TURN = null;

    MMRobot robotInstance;

    final double halfOpenClaw = 0.7;
    final double rotator = 0;
    final double intakeArmPose = 0.59;
    Limelight3A limelight;


    public TrialAutoSample() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        limelight.start();

        Pose2d currentPose = new Pose2d(-39, -65.5, Math.toRadians(180));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);

        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(-48, -65.5), Math.toRadians(180));

        // limelight update

        TrajectoryActionBuilder driveToFirstSample = driveToScorePreloadSample.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-58, -44.7), Math.toRadians(246));

        TrajectoryActionBuilder midLimeLightFirst = driveToFirstSample.endTrajectory().fresh();

        TrajectoryActionBuilder driveToSecondSample = midLimeLightFirst.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-63, -48.3), Math.toRadians(258.9));

        TrajectoryActionBuilder midLimeLightSecond = driveToSecondSample.endTrajectory().fresh();

        TrajectoryActionBuilder driveToIntakeThird = midLimeLightSecond.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50.3, -44), Math.toRadians(315));

        TrajectoryActionBuilder midLimeLightThird = driveToIntakeThird.endTrajectory().fresh();

        TrajectoryActionBuilder driveToScoreThird = midLimeLightThird.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-55.5, -55), Math.toRadians(225));

        TrajectoryActionBuilder driveToPark = driveToScoreThird.endTrajectory().fresh()
                .setTangent(Math.toRadians(65))
                .splineToLinearHeading(new Pose2d(-24, -10, Math.toRadians(0)), Math.toRadians(0));

        new SequentialCommandGroup(
                new InstantCommand(),
                //pre-load
                new ParallelCommandGroup(
                        new ActionCommand(driveToScorePreloadSample.build()),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.MID_POSE),
                                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORE_SAMPLE_POSE)
                                ),
                                new WaitCommand(200),
                                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET), //the height of the high basket
                                new WaitCommand(100),
                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORE_SAMPLE)
                        )
                ),


                //first

                new ActionCommand(driveToFirstSample.build()).alongWith(
                        ScoringSampleCommand.ScoreHighSample(),
                        IntakeSampleCommand.prepareSampleIntake()),
                limelightGetter.getAlignToSampleAuto(limelight, drive, midLimeLightFirst),
//                new WaitCommand(3000),
//                new ActionCommand(driveToFirstSample.build()),
                new InstantCommand(() -> {
                    telemetry.addData("limelight info", LIMELIGHT_INFO);
                    LIMELIGHT_INFO = -1;
                }),
                new LazyActionCommand(() -> LIMELIGHT_TURN.build()),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.maxOpening).alongWith(MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE)),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                ScoringSampleCommand.PrepareHighSample(),
                new WaitCommand(200),
                ScoringSampleCommand.ScoreHighSample(),

                //second
                new ActionCommand(driveToSecondSample.build()).alongWith(
                        IntakeSampleCommand.prepareSampleIntake_Lime()),
                limelightGetter.getAlignToSampleAuto(limelight, drive, midLimeLightSecond).withTimeout(1500),
//                new ActionCommand(driveToSecondSample.build()),
                new InstantCommand(() -> {
                    telemetry.addData("limelight info", LIMELIGHT_INFO);
                    LIMELIGHT_INFO = -1;
                }),
                new LazyActionCommand(() -> LIMELIGHT_TURN.build()),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.maxOpening).alongWith(MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE)),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                ScoringSampleCommand.PrepareHighSample(),
                new WaitCommand(200),
                ScoringSampleCommand.ScoreHighSample(),

                //third
                new ActionCommand(driveToIntakeThird.build()).alongWith(
                        IntakeSampleCommand.prepareSampleIntake_Lime()),
                limelightGetter.getAlignToSampleAuto(limelight, drive, midLimeLightThird).withTimeout(1500),
//                new ActionCommand(driveToIntakeThird.build()),
                new InstantCommand(() -> {
                    telemetry.addData("limelight info", LIMELIGHT_INFO);
                    LIMELIGHT_INFO = -1;
                }),
                new LazyActionCommand(() -> LIMELIGHT_TURN.build()),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.maxOpening),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                new ActionCommand(driveToScoreThird.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample()),

                //park
                new ActionCommand(driveToPark.build()).alongWith(
                        ScoringSampleCommand.ScoreHighSample()
                        //robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PARK_AUTO)
                ),
                new InstantCommand(() -> robotInstance.mmSystems.scoringArm.CutPower())
        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
//        telemetry.addData("linear", MMRobot.getInstance().mmSystems.linearIntake.getPosition());
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }

    public Command setupForPushing() {
        return new ParallelCommandGroup(
//                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.maxOpening),
                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                robotInstance.mmSystems.intakeEndUnitRotator.setPosition(rotator),
                robotInstance.mmSystems.intakEndUnit.setPose(halfOpenClaw)
        );
    }
}