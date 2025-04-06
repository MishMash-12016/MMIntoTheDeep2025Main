package org.firstinspires.ftc.teamcode.CommandGroup.roadRunnewr;

import static org.firstinspires.ftc.teamcode.Autonomous.Red_Right_6.scorePose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Autonomous.Red_Right_6;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;


@Config
public class driveToScoreFirstSpecimen extends CommandBase {

    public static double minAccelMult = 1;
    public static double maxAccelMult = 1.5;
    public static double maxSpeedMult = 0.8;


    Boolean finished = true;
    MecanumDrive.CancelableFollowTrajectoryAction strafeTrajectory;
    public driveToScoreFirstSpecimen() {
        addRequirements(
                MMRobot.getInstance().mmSystems.driveTrain);
    }


    @Override
    public void initialize() {
            Pose2d currentPose = MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR();

            TrajectoryBuilder strafe = MMRobot.getInstance().mmSystems.driveTrain.trajectoryBuilder(currentPose)
                    .setTangent(Math.toRadians(170))
                    .splineToConstantHeading(scorePose.component1(), scorePose.component2(),
                            new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * maxSpeedMult),
                            new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * minAccelMult, MecanumDrive.PARAMS.maxProfileAccel * maxAccelMult));

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