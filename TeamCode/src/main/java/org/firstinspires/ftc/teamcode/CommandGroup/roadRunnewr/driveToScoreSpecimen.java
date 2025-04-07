package org.firstinspires.ftc.teamcode.CommandGroup.roadRunnewr;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.MMRobot;


@Config
public class driveToScoreSpecimen extends CommandBase {

    Boolean finished = true;
    MecanumDrive.CancelableFollowTrajectoryAction strafeTrajectory;
    public driveToScoreSpecimen() {
        addRequirements(
                MMRobot.getInstance().mmSystems.driveTrain);
    }


    @Override
    public void initialize() {
            Pose2d currentPose = MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR();

            TrajectoryBuilder strafe = MMRobot.getInstance().mmSystems.driveTrain.trajectoryBuilder(currentPose)
                    .setTangent(Math.toRadians(260))
                    .splineToSplineHeading(new Pose2d(25, -45, Math.toRadians(140)), Math.toRadians(-10),
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