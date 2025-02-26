package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.LazyActionCommand;
import org.opencv.core.Mat;

import java.util.List;

public class strafeToSample extends CommandBase {
    Limelight3A limelight;

    LLResult result;
    int noResultCounter;
    PinpointDrive drive;
    PIDController pidController;
    TrajectoryActionBuilder previousAction;

    boolean finished;

    public strafeToSample(Limelight3A limelight, PinpointDrive drive) {
        this.limelight = limelight;
        this.drive = drive;
        addRequirements(MMRobot.getInstance().mmSystems.driveTrain);
        finished = false;
    }

    public Pose2d poseForStrafe(Pose2d currentPose, double distance) {
        double newX = currentPose.position.x + distance * Math.cos(currentPose.heading.toDouble());
        double newY = currentPose.position.y + distance * Math.sin(currentPose.heading.toDouble());
        return new Pose2d(newX, newY, currentPose.heading.toDouble());
    }

    @Override
    public void initialize() {
        noResultCounter = 0;
        result = null;
    }

    @Override
    public void execute() {
        limelight.pipelineSwitch(0);
        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            noResultCounter = 0;
            List<LLResultTypes.DetectorResult> allDetectorResults = result.getDetectorResults();
            LLResultTypes.DetectorResult dr = allDetectorResults.get(0);

            Pose2d newPose = poseForStrafe(drive.pose, -dr.getTargetXDegrees());
            TrajectoryActionBuilder strafe = drive.actionBuilder(drive.pose).strafeTo(new Vector2d(newPose.position.x,  newPose.position.y));
            new LazyActionCommand(() -> strafe.build());
            MMRobot.getInstance().mmSystems.telemetry.addData("STRAFED",0);
//          drive.followTrajectoryAsync(traj1);


            MMRobot.getInstance().mmSystems.telemetry.addData("dr",-dr.getTargetXDegrees());
            finished = true;
        }
        else {
            noResultCounter++;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
