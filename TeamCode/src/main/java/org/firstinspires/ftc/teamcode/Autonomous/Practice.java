package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class Practice extends LinearOpMode {
    @Override
    public void runOpMode() {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-60,0,0);

        drive.setPoseEstimate(startPose);

        Trajectory START_POINT_TO_BASCET = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-55, 54,Math.toRadians(-60)), Math.toRadians(90))
                .build();

        Trajectory FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE = drive.trajectoryBuilder(START_POINT_TO_BASCET.end())
                .splineToLinearHeading(new Pose2d(-44,30,Math.toRadians(40)),Math.toRadians(0))
                .build();


        Trajectory FROM_FIRST_INTAKE_TO_SCORING = drive.trajectoryBuilder(FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE.end())
                .splineToLinearHeading(new Pose2d(-55,55,Math.toRadians(-45)),Math.toRadians(97))
                .build();

        Trajectory FROM_SCORING_TO_PARKING = drive.trajectoryBuilder(FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE.end())
                .splineToLinearHeading(new Pose2d(-10,23,Math.toRadians(90)),Math.toRadians(-90))
                .build();





        waitForStart();


        if (!isStopRequested() && opModeIsActive()) {

            drive.followTrajectory(START_POINT_TO_BASCET);

            drive.followTrajectory(FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE);

            drive.followTrajectory(FROM_FIRST_INTAKE_TO_SCORING);

            drive.followTrajectory(FROM_SCORING_TO_PARKING);

        }



    }
}
