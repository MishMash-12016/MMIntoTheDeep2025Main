package org.firstinspires.ftc.teamcode.CommandGroup.roadRunnewr;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Autonomous.Red_Right_6;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.MMRobot;


@Config
public class runFromHereToScore extends CommandBase {

    Boolean finished = true;
    MecanumDrive.CancelableFollowTrajectoryAction strafeTrajectory;
    Pose2d pose;
    public runFromHereToScore(Pose2d pose) {
        this.pose = pose;
        addRequirements(
                MMRobot.getInstance().mmSystems.driveTrain);
    }


    @Override
    public void initialize() {
        Pose2d currentPose = MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR();

        TrajectoryBuilder strafe = MMRobot.getInstance().mmSystems.driveTrain.trajectoryBuilder(currentPose)
                .setTangent(Math.toRadians(140))
                .splineToLinearHeading(pose, Math.toRadians(110),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel*1.2),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel*1.4));

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