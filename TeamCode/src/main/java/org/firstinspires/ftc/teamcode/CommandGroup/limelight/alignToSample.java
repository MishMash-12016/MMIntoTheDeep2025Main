package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.List;

public class alignToSample extends CommandBase {
    Limelight3A limelight;

    LLResult result;
    int noResultCounter;
    PIDController pidController;
    public alignToSample(Limelight3A limelight) {
        this.limelight = limelight;
        limelight.pipelineSwitch(0);
        addRequirements(
                MMRobot.getInstance().mmSystems.driveTrain);
    }


    @Override
    public void initialize() {
        noResultCounter = 0;
        result = null;
        pidController = new PIDController(0.019, 0,0.0005);
        pidController.setSetPoint(0);
        pidController.setTolerance(2);
    }

    @Override
    public void execute() {
        result = limelight.getLatestResult();
        MMRobot.getInstance().mmSystems.telemetry.addData("align to sample",0);

        if (result != null) {
            noResultCounter = 0;
            List<LLResultTypes.DetectorResult> allDetectorResults = result.getDetectorResults();
            LLResultTypes.DetectorResult dr = allDetectorResults.get(0);

            MMRobot.getInstance().mmSystems.driveTrain.drive(0, 0, pidController.calculate(-dr.getTargetXDegrees()));
        }
        else {
            noResultCounter++;
        }
        MMRobot.getInstance().mmSystems.telemetry.update();
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetPoint() || noResultCounter == 5;
    }
}
