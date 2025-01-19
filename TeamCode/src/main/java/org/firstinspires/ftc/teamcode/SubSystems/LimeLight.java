package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.opencv.core.Mat;

import com.fasterxml.jackson.databind.node.DoubleNode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

public class LimeLight extends SubsystemBase {


    public final double maxOpeningLinearCM = 34.5 ;//cm
    public final double heightFromGround = 42; //cm
    public final double angleFixed = 45; //degrees
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

    public Double calculate_distance_vectors(List<Double> vector1,List<Double> vector2){
        return Math.sqrt((vector1.get(0) - vector2.get(0)) * (vector1.get(0) - vector2.get(0)) + (vector1.get(1) - vector2.get(1)) * (vector1.get(1) - vector2.get(1)));
    }

    public String findOrientation(LLResultTypes.DetectorResult result) {
        List<List<Double>> corners =  result.getTargetCorners();
        Double dis1  = calculate_distance_vectors(corners.get(0), corners.get(1));
        Double dis2  = calculate_distance_vectors(corners.get(0), corners.get(3));
        double ratio = dis1 / dis2;
        if (ratio < 1){
            return "-45TO45";
        }
        else if (ratio > 1){
            return "";
        }
        else {
            return "";
        }
    }


}