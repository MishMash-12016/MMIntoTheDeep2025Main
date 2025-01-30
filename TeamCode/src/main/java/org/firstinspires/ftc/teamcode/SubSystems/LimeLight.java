package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
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

    public Command deleteSnapshots(Limelight3A limelight){
        return new InstantCommand(()->
                limelight.deleteSnapshots()
        );
    }

    public Command gotoSample(Limelight3A limelight) {
        PIDController pidController = new PIDController(0, 0, 0);
        pidController.setSetPoint(0);
        pidController.setTolerance(0.5);

        return new RunCommand(
                () -> {
                    LLResult result = limelight.getLatestResult();
                    //capture the snapshots
                    limelight.deleteSnapshots();
                    limelight.captureSnapshot("sharabi");

                    if (result != null && result.isValid()) {
                        List<LLResultTypes.DetectorResult> allDetectorResults = result.getDetectorResults();
                        LLResultTypes.DetectorResult dr = allDetectorResults.get(0);
//                        //Rotate claw, then rotate robot to sample, and then open linear intake
                          rotateClawToSample(limelight,dr);
//                        MMRobot.getInstance().mmSystems.driveTrain.drive(0, 0, pidController.calculate(dr.getTargetYDegrees()));
//                        openLinearToSample(dr);
                    }

                }, this); // do set position void or else wont work
//                .interruptOn(()-> pidController.atSetPoint() && limelight.getLatestResult().isValid());
    }

//    public void openLinearToSample(LLResultTypes.DetectorResult result) {
//        MMRobot.getInstance().mmSystems.linearIntake.setPosition(
//                calculateDistance(result.getTargetYDegrees() / maxOpeningLinearCM * LinearIntake.maxOpening));
//    }

    public Double calculate_distance_vectors(List<Double> vector1,List<Double> vector2){
        return Math.sqrt((vector1.get(0) - vector2.get(0)) * (vector1.get(0) - vector2.get(0)) + (vector1.get(1) - vector2.get(1)) * (vector1.get(1) - vector2.get(1)));
    }

    public void rotateClawToSample(Limelight3A limelight ,LLResultTypes.DetectorResult result){
        double angle = getSampleAngle(limelight, result);
        double angleInServoDegrees = angle / 270;
        MMRobot.getInstance().mmSystems.telemetry.addData("angle = ", angle);
        MMRobot.getInstance().mmSystems.telemetry.addData("angle for servo= ",angleInServoDegrees);
        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPositionVoid(angleInServoDegrees);
    }

    public double getSampleAngle(Limelight3A limelight ,LLResultTypes.DetectorResult result){
        List<List<Double>> corners =  result.getTargetCorners();
        List<Double> cornerUpLeft = corners.get(0);
        List<Double> cornerUpRight = corners.get(1);
        List<Double> cornerDownLeft = corners.get(3);
        Double height  = calculate_distance_vectors(cornerUpLeft, cornerDownLeft);
        Double width  = calculate_distance_vectors(cornerUpLeft, cornerUpRight);
        limelight.pipelineSwitch(1);
        //crop_x, crop_y, crop_width, crop_height = llrobot[0:4] first 4 to send
        double[] inputsPython = {cornerUpLeft.get(0),cornerUpLeft.get(1), width, height};
        limelight.updatePythonInputs(inputsPython);
        double[] outputPython = limelight.getLatestResult().getPythonOutput();

        double angle = outputPython[0];
        double cropped = outputPython[1];
        for (int i =0; i<outputPython.length;i++){
            MMRobot.getInstance().mmSystems.telemetry.addData("output - ",outputPython[i]);
        }
        MMRobot.getInstance().mmSystems.telemetry.addData("width - ",width);
        MMRobot.getInstance().mmSystems.telemetry.addData("height -  ",height);
        MMRobot.getInstance().mmSystems.telemetry.addData("X left up-  ",cornerUpLeft.get(0));
        MMRobot.getInstance().mmSystems.telemetry.addData("Y left up -  ",cornerUpLeft.get(1));
        MMRobot.getInstance().mmSystems.telemetry.addData("did it cropped? ",cropped);
        limelight.pipelineSwitch(0);
        return angle;
    }
}