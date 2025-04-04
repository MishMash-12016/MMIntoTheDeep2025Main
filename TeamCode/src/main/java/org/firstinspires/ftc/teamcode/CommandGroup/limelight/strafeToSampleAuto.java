package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.MMRobot;


@Config
public class strafeToSampleAuto extends CommandBase {
    MecanumDrive.CancelableFollowTrajectoryAction strafeTrajectory;

    public static double maxDistanceY = 470;
    public static double plusDistanceX = 1.5;
    Boolean finished = true;

    Boolean found = true;
    public strafeToSampleAuto() {
        addRequirements(
                MMRobot.getInstance().mmSystems.driveTrain);

        this.strafeTrajectory = strafeTrajectory;
    }


    @Override
    public void initialize() {}

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