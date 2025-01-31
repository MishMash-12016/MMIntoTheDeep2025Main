package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSpecimanCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class Autopushing3 extends MMOpMode {
    MMRobot robotInstance;
    public Autopushing3() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        Pose2d currentPose = (new Pose2d(5.5, -65.5, Math.toRadians(90.00)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);

        TrajectoryActionBuilder driveToScorePreloadSpecimen = drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-5, -27, Math.toRadians(90)), Math.toRadians(90)); //preload
        TrajectoryActionBuilder driveToScorePreloadSpecimen2 =driveToScorePreloadSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-5,-45, Math.toRadians(90)), Math.toRadians(270));
        TrajectoryActionBuilder driveToPushSampleToHuman = driveToScorePreloadSpecimen2.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(38, -44.84, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(38,-14, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(46,-14 , Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(46,-57,Math.toRadians(90)), Math.toRadians(270));
        TrajectoryActionBuilder driveToPushSample2ToHuman = driveToPushSampleToHuman.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(46,-14,Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(55,-14 ,Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(55,-57 ,Math.toRadians(90)), Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(55,-14  ,Math.toRadians(90)), Math.toRadians(90));
        TrajectoryActionBuilder driveToPushSample3ToHuman = driveToPushSample2ToHuman.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(62,-14  ,Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(270))  //pick up first
                .splineToSplineHeading(new Pose2d(62, -67, Math.toRadians(90)), Math.toRadians(270));
        TrajectoryActionBuilder driveToScoreSpecimen1 = driveToPushSample3ToHuman.endTrajectory().fresh()
                .setTangent(160)
                .splineToSplineHeading(new Pose2d(-5, -28, Math.toRadians(90)), Math.toRadians(90));
        TrajectoryActionBuilder driveToScoreSpecimen2 = driveToScoreSpecimen1.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(-5, -45, Math.toRadians(90)), Math.toRadians(90));

        new SequentialCommandGroup(
                new ActionCommand(driveToScorePreloadSpecimen.build()).alongWith(
                ScoringSpecimanCommand.SpecimanScore()),
                new ActionCommand(driveToScorePreloadSpecimen2.build()),
                new ActionCommand(driveToPushSampleToHuman.build()),
                new ActionCommand(driveToPushSample2ToHuman.build()),
                new ActionCommand(driveToPushSample3ToHuman.build()),
                new ActionCommand(driveToScoreSpecimen1.build()),
                new ActionCommand(driveToScoreSpecimen2.build())


        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.addData("linear",MMRobot.getInstance().mmSystems.linearIntake.getPosition());
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }

}
