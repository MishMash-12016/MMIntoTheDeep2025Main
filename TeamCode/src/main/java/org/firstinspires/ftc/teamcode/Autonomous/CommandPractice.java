package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.TrajectroryFollowerCommend;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.AutoMecanumDrive;

public class CommandPractice extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

    final AutoMecanumDrive drive = new AutoMecanumDrive(new SampleMecanumDrive(MMRobot.getInstance().mmSystems.hardwareMap));


        Pose2d startPose = new Pose2d(-60,0,0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence START_POINT_TO_BASCET = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-55, 54,Math.toRadians(-60)), Math.toRadians(90))
                .build();

        TrajectorySequence FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE = drive.trajectorySequenceBuilder(START_POINT_TO_BASCET.end())
                .splineToLinearHeading(new Pose2d(-44,30,Math.toRadians(40)),Math.toRadians(0))
                .build();

        TrajectorySequence FROM_FIRST_INTAKE_TO_SCORING = drive.trajectorySequenceBuilder(FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE.end())
                .splineToLinearHeading(new Pose2d(-55,55,Math.toRadians(-45)),Math.toRadians(97))
                .build();

        TrajectorySequence FROM_SCORING_TO_PARKING = drive.trajectorySequenceBuilder(FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE.end())
                .splineToLinearHeading(new Pose2d(-10,23,Math.toRadians(90)),Math.toRadians(-90))
                .build();



        waitForStart();

        if (!isStopRequested() && opModeIsActive()) {

            new TrajectroryFollowerCommend(START_POINT_TO_BASCET, drive)
                    .schedule();

            new TrajectroryFollowerCommend(FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE, drive)
                    .schedule();

            new TrajectroryFollowerCommend(FROM_FIRST_INTAKE_TO_SCORING,drive)
                    .schedule();

            new TrajectroryFollowerCommend(FROM_SCORING_TO_PARKING,drive)
                    .schedule();



        }

    }
}