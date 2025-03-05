package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.acmerobotics.roadrunner.PoseVelocity2d;
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
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.FTCTimer;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.utils.SQPIDController;
import org.opencv.core.Mat;

import java.util.List;

public class strafeToSampleAuto extends CommandBase {

    SQPIDController pidController;
    FTCTimer timer;
    PinpointDrive drive;
    boolean finished;

    public strafeToSampleAuto(PinpointDrive drive) {
        this.drive = drive;
        addRequirements(MMRobot.getInstance().mmSystems.driveTrain);
        finished = false;
    }


    @Override
    public void initialize() {
        pidController = new SQPIDController(0.0124, 0,0.0013);
        pidController.setSetpoint(0);
        pidController.setTolerance(7);
        timer = new FTCTimer();
        timer.start();
        MMRobot.getInstance().mmSystems.vision.startTracking();
    }

    @Override
    public void execute() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(pidController.calculate(MMRobot.getInstance().mmSystems.vision.getStrafeOffset()),0) , 0));
        if (!pidController.atSetpoint()){
            timer.start();
        }
        MMRobot.getInstance().mmSystems.telemetry.addData("timer", timer.getTime());
    }

    @Override
    public void end(boolean interrupted) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0) , 0));
        MMRobot.getInstance().mmSystems.vision.stopTracking();
    }

    @Override
    public boolean isFinished() {
        timer.end();
        return timer.getTime() >= 300;

    }
}
