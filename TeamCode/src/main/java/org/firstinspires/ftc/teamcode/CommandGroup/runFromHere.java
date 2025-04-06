package org.firstinspires.ftc.teamcode.CommandGroup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Translation2d;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.utils.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Rotation2d;



@Config
public class runFromHere extends CommandBase {

    Boolean finished = true;
    MecanumDrive.CancelableFollowTrajectoryAction strafeTrajectory;
    public runFromHere() {
        addRequirements(
                MMRobot.getInstance().mmSystems.driveTrain);
    }


    @Override
    public void initialize() {
            Pose2d currentPose = MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR();

            TrajectoryBuilder strafe = MMRobot.getInstance().mmSystems.driveTrain.trajectoryBuilder(currentPose)
                    .setTangent(Math.toRadians(260))
                    .splineToLinearHeading(new Pose2d(25, -45, Math.toRadians(140)), Math.toRadians(-10),
                            new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.3),
                            new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));

        strafeTrajectory = MMRobot.getInstance().mmSystems.driveTrain.getCancelableFollowTrajectoryAction(strafe.build().get(0));
    }

    @Override
    public void execute() {
            TelemetryPacket packet = new TelemetryPacket();
            finished = !strafeTrajectory.run(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public void end(boolean interrupted) {
            strafeTrajectory.cancelAbruptly();
            TelemetryPacket packet = new TelemetryPacket();
            strafeTrajectory.run(packet);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}