package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Autonomous.ActionCommand;
import org.firstinspires.ftc.teamcode.MMRobot;

@Config
public class strafeToSample extends CommandBase {
    public strafeToSample() {
        addRequirements(
                MMRobot.getInstance().mmSystems.driveTrain);
    }


    @Override
    public void initialize() {
        MMRobot.getInstance().mmSystems.vision.startTracking();
    }

    @Override
    public void execute() {
        double distanceX = MMRobot.getInstance().mmSystems.vision.getStrafeOffset();
        if (distanceX != 0) {
            Pose2d currentPose = MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR();
            TrajectoryActionBuilder strafe = MMRobot.getInstance().mmSystems.driveTrain.actionBuilder(currentPose)
                    .strafeTo(new Vector2d(currentPose.component1().x + Math.cos(currentPose.heading.toDouble() + Math.toRadians(90)) * distanceX ,currentPose.component1().y + Math.sin(currentPose.heading.toDouble() + Math.toRadians(90)) * distanceX));
            ActionCommand driveCommand = new ActionCommand(strafe.build());
//            driveCommand.interruptOn(()->false);
            driveCommand.schedule();
        }
        FtcDashboard.getInstance().getTelemetry().update();
    }


    @Override
    public void end(boolean interrupted) {
        MMRobot.getInstance().mmSystems.vision.stopTracking();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}