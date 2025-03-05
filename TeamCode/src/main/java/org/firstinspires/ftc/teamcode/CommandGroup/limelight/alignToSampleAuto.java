package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Autonomous.TrialAutoSample;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.SQPIDController;
import org.opencv.core.Mat;

import java.util.List;

public class alignToSampleAuto extends CommandBase {
    Limelight3A limelight;

    LLResult result;
    int noResultCounter;
    PinpointDrive drive;
    TrajectoryActionBuilder previousAction;
    boolean finished;

    public alignToSampleAuto(Limelight3A limelight, PinpointDrive drive, TrajectoryActionBuilder previousAction) {
        this.limelight = limelight;
        this.drive = drive;
        addRequirements(MMRobot.getInstance().mmSystems.driveTrain);
        this.previousAction = previousAction;
        finished = false;
    }


    @Override
    public void initialize() {
        noResultCounter = 0;
        result = null;
        MMRobot.getInstance().mmSystems.vision.startTracking();
    }

    @Override
    public void execute() {
        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            noResultCounter = 0;
            List<LLResultTypes.DetectorResult> allDetectorResults = result.getDetectorResults();
            LLResultTypes.DetectorResult dr = allDetectorResults.get(0);
            TrialAutoSample.LIMELIGHT_TURN = previousAction.turn(Math.toRadians(-dr.getTargetXDegrees()));
            TrialAutoSample.LIMELIGHT_INFO = -dr.getTargetXDegrees();
            MMRobot.getInstance().mmSystems.telemetry.addData("dr",-dr.getTargetXDegrees());
            finished = true;
        }
        else {
            noResultCounter++;
        }
    }

    @Override
    public void end(boolean interrupted) {
        MMRobot.getInstance().mmSystems.vision.stopTracking();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
