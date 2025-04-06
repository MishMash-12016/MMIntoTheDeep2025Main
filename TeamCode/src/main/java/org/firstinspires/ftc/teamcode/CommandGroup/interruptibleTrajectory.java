package org.firstinspires.ftc.teamcode.CommandGroup;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Rotation2d;

import java.util.function.BooleanSupplier;


@Config
public class interruptibleTrajectory extends CommandBase {
    MecanumDrive.CancelableFollowTrajectoryAction cancellableTrajectory;
    Boolean finished = true;
    Vector2d pose2d;
    double tangent;
    VelConstraint velConstraint;
    AccelConstraint accelConstraint;
    Double setTangent;
    boolean forceStopped;
    BooleanSupplier flag;

    public interruptibleTrajectory(BooleanSupplier flag , Vector2d pose2d, double tangent, VelConstraint velConstraint, AccelConstraint accelConstraint , Double setTangent) {
        this.flag = flag;
        this.pose2d = pose2d;
        this.tangent = tangent;
        this.velConstraint = velConstraint;
        this.accelConstraint = accelConstraint;
        this.setTangent = setTangent;
        addRequirements(
                MMRobot.getInstance().mmSystems.driveTrain);
    }

    @Override
    public void initialize() {
        TrajectoryBuilder trajectory = MMRobot.getInstance().mmSystems.driveTrain.trajectoryBuilder(MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR())
                .splineToConstantHeading(pose2d, tangent, velConstraint, accelConstraint);
        if (setTangent != null){
            trajectory.setTangent(setTangent);
        }

        cancellableTrajectory = MMRobot.getInstance().mmSystems.driveTrain.getCancelableFollowTrajectoryAction(trajectory.build().get(0));
        finished = false;

    }

    @Override
    public void execute() {
        if (flag.getAsBoolean()){
                finished = true;
                forceStopped = true;
        }
        else {
            TelemetryPacket packet = new TelemetryPacket();
            finished = !cancellableTrajectory.run(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public void end(boolean interrupted) {
        if (forceStopped){
            cancellableTrajectory.cancelAbruptly();
            TelemetryPacket packet = new TelemetryPacket();
            cancellableTrajectory.run(packet);
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}