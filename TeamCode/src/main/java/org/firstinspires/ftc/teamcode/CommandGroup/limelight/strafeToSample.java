package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Rotation2d;



@Config
public class strafeToSample extends CommandBase {
    MecanumDrive.CancelableFollowTrajectoryAction strafeTrajectory;

    public static double maxDistanceY = 440;
    public static double xOffset = 0;
    Boolean finished = true;

    Boolean found = false;
    public strafeToSample() {
        addRequirements(
                MMRobot.getInstance().mmSystems.driveTrain);
    }


    @Override
    public void initialize() {
        double distanceX = MMRobot.getInstance().mmSystems.vision.getStrafeOffset() + xOffset;
        double distanceY = (maxDistanceY - MMRobot.getInstance().mmSystems.vision.getDistance())/25.4;

        if (distanceX != 0) {
            Pose2d currentPose = MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR();

            Translation2d distanceXVector = new Translation2d(distanceX,new Rotation2d(currentPose.heading.toDouble() + Math.toRadians(90)));
            Translation2d distanceYVector = new Translation2d(distanceY, new Rotation2d(currentPose.heading.toDouble()));
            Translation2d endPoint = new Translation2d(currentPose.position.x, currentPose.position.y)
                    .plus(distanceXVector)
                    .plus(distanceYVector);

            TrajectoryBuilder strafe = MMRobot.getInstance().mmSystems.driveTrain.trajectoryBuilder(currentPose)
                    .strafeTo(new Vector2d(endPoint.getX(), endPoint.getY()),
                            new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 0.5), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.5,MecanumDrive.PARAMS.maxProfileAccel * 0.5) );

            strafeTrajectory = MMRobot.getInstance().mmSystems.driveTrain.getCancelableFollowTrajectoryAction(strafe.build().get(0));
            found = true;
        }
        else {
            found = false;
            finished = true;
        }
    }

    @Override
    public void execute() {
        if (found){
            TelemetryPacket packet = new TelemetryPacket();
            finished = !strafeTrajectory.run(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public void end(boolean interrupted) {
        if (found){
            strafeTrajectory.cancelAbruptly();
            TelemetryPacket packet = new TelemetryPacket();
            strafeTrajectory.run(packet);
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}