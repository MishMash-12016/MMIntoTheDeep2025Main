package org.firstinspires.ftc.teamcode.Autonomous;

//imports

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;


@Autonomous
public class AutoSample1 extends MMOpMode {
    MMRobot robotInstance;
    int waitBeforeOpeningScoringClawTime = 1000;

    public AutoSample1() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();
        Pose2d currentPose = (new Pose2d(-19, -66.28, Math.toRadians(90.00)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);
        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORE_SAMPLE_POSE);

        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(140))
                .splineToLinearHeading(new Pose2d(-52.8, -59, Math.toRadians(225)), Math.toRadians(250)); //
        TrajectoryActionBuilder driveToPickUpFirstSample = driveToScorePreloadSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(-49, -51.25, Math.toRadians(270)), Math.toRadians(80));
        TrajectoryActionBuilder driveToScoreFirstSample = driveToPickUpFirstSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-52.8, -59, Math.toRadians(215)), Math.toRadians(250)); //score
        TrajectoryActionBuilder driveToPark = driveToScoreFirstSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-51.8, -50, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(-60);


        new SequentialCommandGroup(
                new ActionCommand(driveToScorePreloadSample.build()),
                ScoringSampleCommand.PrepareHighSample(),
                new WaitCommand(200),
                ScoringSampleCommand.ScoreHighSample(),
                new WaitCommand(500),
                new ActionCommand(driveToPickUpFirstSample.build()),
                new WaitCommand(200),
                IntakeSampleCommand.prepareSampleIntake(()-> false,()-> false).withTimeout(600),
                new WaitCommand(200),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                new ActionCommand(driveToScoreFirstSample.build()),
                ScoringSampleCommand.PrepareHighSample(),
                new WaitCommand(200),
                ScoringSampleCommand.ScoreHighSample(),
                new WaitCommand(400),
                new ActionCommand(driveToPark.build())
        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
    }

}





