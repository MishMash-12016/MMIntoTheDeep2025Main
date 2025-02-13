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

public class alignToSampleAuto extends CommandBase {
    Limelight3A limelight;

    LLResult result;
    int noResultCounter;
    PIDController pidController;
    PinpointDrive drive;

    public alignToSampleAuto(Limelight3A limelight , PinpointDrive drive) {
        this.limelight = limelight;
        this.drive = drive;
        addRequirements(MMRobot.getInstance().mmSystems.driveTrain);
    }


    @Override
    public void initialize() {
        noResultCounter = 0;
        result = null;
        pidController = new PIDController(0.017, 0, 0.0005);
        pidController.setSetPoint(0);
        pidController.setTolerance(1);
    }

    @Override
    public void execute() {
        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            noResultCounter = 0;
            List<LLResultTypes.DetectorResult> allDetectorResults = result.getDetectorResults();
            LLResultTypes.DetectorResult dr = allDetectorResults.get(0);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),pidController.calculate(-dr.getTargetXDegrees())));
        }
        else {
            noResultCounter++;
        }
    }

    @Override
    public boolean isFinished() {
        return noResultCounter > 5 || pidController.atSetPoint();
    }
}
