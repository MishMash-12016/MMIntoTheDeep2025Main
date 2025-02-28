package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Autonomous.AutoSpecimen;
import org.firstinspires.ftc.teamcode.Autonomous.TrialAutoSample;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.core.Mat;

import java.util.List;

public class strafeToSampleAuto extends CommandBase {
    Limelight3A limelight;

    LLResult result;
    int noResultCounter;
    PinpointDrive drive;
    PIDController pidController;
    TrajectoryActionBuilder previousAction;
    boolean finished;

    private double robotWidth = 5.5;

    public strafeToSampleAuto(Limelight3A limelight, PinpointDrive drive, TrajectoryActionBuilder previousAction) {
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
        limelight.pipelineSwitch(1);
    }

    @Override
    public void execute() {

            double[] outputPython = limelight.getLatestResult().getPythonOutput();
            double limelight_x =  outputPython[1];
            AutoSpecimen.LIMELIGHT_TURN = previousAction.strafeToLinearHeading(new Vector2d(drive.pose.position.x + limelight_x / 2.54 + robotWidth ,drive.pose.position.y), Math.toRadians(270));
            AutoSpecimen.LIMELIGHT_INFO = limelight_x;
            MMRobot.getInstance().mmSystems.telemetry.addData("dr",limelight_x);
            finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
