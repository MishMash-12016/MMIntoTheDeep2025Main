package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.MMRobot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

public class LimeLight extends SubsystemBase {
    private Limelight3A limelight;

    public final double maxOpeningLinearCM = 34.5;//cm
    public final double heightFromGround = 11.9; //cm
    public final double angleFixed = 0; //degrees
    public final double sampleHeight = 3.9; //cm


    public LimeLight() {

    }

    public double calculateDistance(double angleFromLimelight) {
        double height = heightFromGround - sampleHeight;
        double angle = Math.abs(angleFromLimelight + angleFixed);
        return height / Math.tan(Math.toRadians(angle));
    }

    public Command gotoSample(Limelight3A limelight) {
        PIDController pidController = new PIDController(0, 0, 0);
        pidController.setSetPoint(0);
        pidController.setTolerance(0.5);

        return new RunCommand(
                () -> {
                    LLResult result = limelight.getLatestResult();

                    if (result != null && result.isValid()) {
                        List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                        for (LLResultTypes.DetectorResult dr : detectorResults) {
                            MMRobot.getInstance().mmSystems.driveTrain.drive(0, 0, pidController.calculate(dr.getTargetYDegrees()));
                            openLinearToSample(dr);}
                    }
                    else {
                        MMRobot.getInstance().mmSystems.driveTrain.drive(0,0,0);
                    }
                }, this)
                .interruptOn(()-> pidController.atSetPoint() && limelight.getLatestResult().isValid());
    }

    public void openLinearToSample(LLResultTypes.DetectorResult result) {
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(
                calculateDistance(result.getTargetYDegrees() / maxOpeningLinearCM * LinearIntake.maxOpening));
    }


}