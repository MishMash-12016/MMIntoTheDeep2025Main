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

//        MMRobot.getInstance().init(OpModeType.NonCompetition.EXPERIMENTING, hardwareMap, gamepad1,gamepad2,telemetry);


        /* TODO:
            hello itay! writing this in 19/10/24 (2:40am)
            to resolve this issue, try to run the same code without AutoMecanumDrive (like before)
            if the issue is still there, then there's some other problem in the code, try to comment lines, and see what effects it.
            if it works, then u know the issue is in AutoMecanumDrive or the implementation of it here.
            in this case, u could try and debug it, slowly commenting lines and finding the problem.
            or u could just take the AutoDriveTrain from last year.
            good luck!
         */

        AutoMecanumDrive drive = new AutoMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-60,0,0);

        drive.getDrive().setPoseEstimate(startPose);

        Trajectory START_POINT_TO_BASCET = drive.getDrive().trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-55, 54,Math.toRadians(-60)), Math.toRadians(90))
                .build();

        Trajectory FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE = drive.getDrive().trajectoryBuilder(START_POINT_TO_BASCET.end())
                .splineToLinearHeading(new Pose2d(-44,30,Math.toRadians(40)),Math.toRadians(0))
                .build();

        waitForStart();

        if (!isStopRequested() && opModeIsActive()) {

            drive.followTrajectory(START_POINT_TO_BASCET);

            drive.followTrajectory(FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE);

        }

//        MMRobot.getInstance().resetRobot();

    }
}
