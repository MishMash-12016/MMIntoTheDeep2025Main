package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Commands.TrajectroryFollowerCommend;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.AutoMecanumDrive;

import java.util.List;

public class CommandPractice extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

    final AutoMecanumDrive drive = new AutoMecanumDrive(new SampleMecanumDrive(MMRobot.getInstance().mmSystems.hardwareMap));

        Pose2d startPose = new Pose2d(-60,0,0);

        drive.setPoseEstimate(startPose);

        Trajectory START_POINT_TO_BASCET = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-55, 54,Math.toRadians(-60)), Math.toRadians(90))
                .build();

        Trajectory FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE = drive.trajectoryBuilder(START_POINT_TO_BASCET.end())
                .splineToLinearHeading(new Pose2d(-44,30,Math.toRadians(40)),Math.toRadians(0))
                .build();

        waitForStart();

        if (!isStopRequested() && opModeIsActive()) {

//            new TrajectroryFollowerCommend(drive.trajectorySequenceBuilder(new Pose2d()));

//            drive.followTrajectory(START_POINT_TO_BASCET);
//
//            drive.followTrajectory(FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE);

        }


    }
}