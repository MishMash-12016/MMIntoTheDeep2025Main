package org.firstinspires.ftc.teamcode.Autonomous;

//imports

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSpecimanCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

import java.util.Collections;


@Autonomous
public class autoCloseRed extends MMOpMode {
    MMRobot robotInstance;

    public autoCloseRed() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();
        Pose2d currentPose = (new Pose2d(-5.5, -65.5, Math.toRadians(90.00)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load

        TrajectoryActionBuilder driveToScorePreloadSpecimen = drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(-5, -33, Math.toRadians(90)), Math.toRadians(90));
        TrajectoryActionBuilder driveToScorePreLoadSpecimen2 = driveToScorePreloadSpecimen.endTrajectory().fresh()
                .lineToY(-50);
        TrajectoryActionBuilder driveToPickUpFirstSample = driveToScorePreLoadSpecimen2.endTrajectory().fresh()
                .setTangent(Math.toRadians(240))
                .splineToLinearHeading(new Pose2d(-48.84, -44.84, Math.toRadians(270)), Math.toRadians(180));
        TrajectoryActionBuilder drivetToScoreFirstSample = driveToPickUpFirstSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(240))
                .splineToLinearHeading(new Pose2d(-55.97, -59.18, Math.toRadians(225)), Math.toRadians(270));
        TrajectoryActionBuilder driveToPickUpSecondSample = drivetToScoreFirstSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(-57.875, -44.84, Math.toRadians(270)), Math.toRadians(95));
        TrajectoryActionBuilder driveToScoreSecondSample = driveToPickUpSecondSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(275))
                .splineToLinearHeading(new Pose2d(-55.97, -59.18, Math.toRadians(225)), Math.toRadians(270));
        TrajectoryActionBuilder driveToPickUpThirdSample = driveToScoreSecondSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(-50.8, -45.8, Math.toRadians(315)), Math.toRadians(95));
        TrajectoryActionBuilder driveToScoreThirdSample = driveToPickUpThirdSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-55.97, -59.18, Math.toRadians(225)), Math.toRadians(270));

        new SequentialCommandGroup(
                new ActionCommand(driveToScorePreloadSpecimen.build()),
                ScoringSpecimanCommand.SpecimanScore(),
                new WaitCommand(500),
                new ActionCommand(driveToScorePreLoadSpecimen2.build())
                        .alongWith(new WaitCommand(400).andThen(MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw())),

                new ActionCommand(driveToPickUpFirstSample.build())
        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        FtcDashboard.getInstance().getTelemetry().addData("height", MMRobot.getInstance().mmSystems.elevator.getHeight());
        FtcDashboard.getInstance().getTelemetry().addData("target pose:", MMRobot.getInstance().mmSystems.elevator.targetPose);
        FtcDashboard.getInstance().getTelemetry().update();
    }

}
