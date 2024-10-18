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

        AutoMecanumDrive drive = new AutoMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-60,0,0);

        drive.getDrive().setPoseEstimate(startPose);

        waitForStart();

        if (!isStopRequested() && opModeIsActive()) {

            Trajectory START_POINT_TO_BASCET = drive.getDrive().trajectoryBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-55, 54,Math.toRadians(-60)), Math.toRadians(90))
                    .build();

            drive.followTrajectory(START_POINT_TO_BASCET);



            Trajectory FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE = drive.getDrive().trajectoryBuilder(START_POINT_TO_BASCET.end())
                    .splineToLinearHeading(new Pose2d(-44,30,Math.toRadians(40)),Math.toRadians(0))
                    .build();

            drive.followTrajectory(FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE);

        }

//        MMRobot.getInstance().resetRobot();

    }
}
