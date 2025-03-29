package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.MMRobot;

@Config
public class strafeToSample extends CommandBase {
    MecanumDrive.CancelableFollowTrajectoryAction strafeTrajectory;

    Boolean finished = true;
    public strafeToSample() {
        addRequirements(
                MMRobot.getInstance().mmSystems.driveTrain);
    }


    @Override
    public void initialize() {
        MMRobot.getInstance().mmSystems.vision.startTracking();

        double distanceX = -MMRobot.getInstance().mmSystems.vision.getStrafeOffset();
        if (distanceX != 0) {
            Pose2d currentPose = MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR();
            TrajectoryBuilder strafe = MMRobot.getInstance().mmSystems.driveTrain.trajectoryBuilder(currentPose)
                    .strafeTo(new Vector2d(currentPose.component1().x + Math.cos(currentPose.heading.toDouble() + Math.toRadians(90)) * distanceX ,currentPose.component1().y + Math.sin(currentPose.heading.toDouble() + Math.toRadians(90)) * distanceX));

            strafeTrajectory = MMRobot.getInstance().mmSystems.driveTrain.getCancelableFollowTrajectoryAction(strafe.build().get(0));
        }
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        finished = !strafeTrajectory.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public void end(boolean interrupted) {
        MMRobot.getInstance().mmSystems.vision.stopTracking();
        strafeTrajectory.cancelAbruptly();
        TelemetryPacket packet = new TelemetryPacket();
        strafeTrajectory.run(packet);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}