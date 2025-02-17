package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class TrialAutoSample extends MMOpMode {
    MMRobot robotInstance;

    final double halfOpenClaw = 0.7;
    final double rotator = 0;
    final double intakeArmPose = 0.59;

    public TrialAutoSample() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        Pose2d currentPose = new Pose2d(-39, -65.5, Math.toRadians(180));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);

        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(-48, -65.5), Math.toRadians(180));

        TrajectoryActionBuilder driveToIntakeFirstSample = driveToScorePreloadSample.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-58.4, -46.3), Math.toRadians(-112.3));
        TrajectoryActionBuilder driveToScoreFirstSample =  driveToIntakeFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-59.1, -52), Math.toRadians(-112.3));

        TrajectoryActionBuilder driveToSecondSample = driveToScoreFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-62.4, -48), Math.toRadians(-102.56));

        TrajectoryActionBuilder driveToIntakeThird = driveToSecondSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-62, -45.78), Math.toRadians(-75.16));
        TrajectoryActionBuilder driveToScoreThird = driveToIntakeThird.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-64.9, -49), Math.toRadians(-100.67));

        TrajectoryActionBuilder driveToPark = driveToScoreThird.endTrajectory().fresh()
                .setTangent(Math.toRadians(67))
                .splineToLinearHeading(new Pose2d(-24, -10, Math.toRadians(0)), Math.toRadians(0));

        new SequentialCommandGroup(
                new InstantCommand(),
                //pre-load
                new ActionCommand(driveToScorePreloadSample.build()),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.MID_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORE_SAMPLE_POSE)
                ),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET), //the height of the high basket
                new WaitCommand(100),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORE_SAMPLE),
                new WaitCommand(200),
                ScoringSampleCommand.ScoreHighSample(),


                //first
                new ActionCommand(driveToIntakeFirstSample.build()).alongWith(
                        new WaitCommand(700).andThen(
                                IntakeSampleCommand.prepareSampleIntake(),
                                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.4)
                        )
                ),
                new WaitCommand(100),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                new ActionCommand(driveToScoreFirstSample.build()),
                ScoringSampleCommand.PrepareHighSample(),
                new WaitCommand(200),
                ScoringSampleCommand.ScoreHighSample(),

                //second
                new ActionCommand(driveToSecondSample.build()).alongWith(
                        IntakeSampleCommand.prepareSampleIntake(),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.35)
                ),
                new WaitCommand(100),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                ScoringSampleCommand.PrepareHighSample(),
                new WaitCommand(200),
                ScoringSampleCommand.ScoreHighSample(),

                //third
                new ActionCommand(driveToIntakeThird.build()).alongWith(
                        IntakeSampleCommand.prepareSampleIntake(),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.25)
                ),
                new WaitCommand(100),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                new ActionCommand(driveToScoreThird.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample()),
                new WaitCommand(200),
                ScoringSampleCommand.ScoreHighSample(),
                new WaitCommand(200),

                //park
                new ActionCommand(driveToPark.build()),
                robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PARK_AUTO),
                new WaitCommand(400),
                new InstantCommand(() -> robotInstance.mmSystems.scoringArm.CutPower())
        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }



}