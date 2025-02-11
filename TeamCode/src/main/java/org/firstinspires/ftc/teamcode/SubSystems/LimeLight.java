package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

public class LimeLight extends SubsystemBase {


    public final double maxOpeningLinearCM = 33.5 ;//cm
    public final double heightFromGround = 42; //cm
    public final double angleFixed = 45; //degrees
    public final double sampleHeight = 3.9; //cm
    public final double armLength = 15; //cm


    public double oldAngle = 0;


    public LimeLight() {

    }

    public double calculateDistance(double angleFromLimelight) {
        double height = heightFromGround - sampleHeight;
        double angle = Math.abs(angleFromLimelight + angleFixed);
        return height / Math.tan(Math.toRadians(angle));
    }

    public void changeOriention(Limelight3A limelight){
        PIDController pidController = new PIDController(0.017, 0, 0.0005);
        pidController.setSetPoint(0);
        pidController.setTolerance(5);

        while (!pidController.atSetPoint())
                {
                    LLResult result = limelight.getLatestResult();
                    if (result != null && result.isValid()) {

                        List<LLResultTypes.DetectorResult> allDetectorResults = result.getDetectorResults();
                        LLResultTypes.DetectorResult dr = allDetectorResults.get(0);
                        MMRobot.getInstance().mmSystems.driveTrain.drive(0, 0, pidController.calculate(-dr.getTargetXDegrees()));
                    }
                }
    }

    public void changeOrientionAuto(Limelight3A limelight , PinpointDrive drive , Pose2d currentPose){
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {

                List<LLResultTypes.DetectorResult> allDetectorResults = result.getDetectorResults();
                LLResultTypes.DetectorResult dr = allDetectorResults.get(0);
                drive.actionBuilder(currentPose).setTangent(-dr.getTargetXDegrees());
            }

    }

    public Command turnToSample(Limelight3A limelight,  PinpointDrive drive , Pose2d currentPose){
        return new InstantCommand(
                () -> {
                    changeOrientionAuto(limelight, drive, currentPose);
                }, this);
    }

    public Command gotoSample(Limelight3A limelight) {

        return new InstantCommand(
                () -> {
//                    while (angle == 0 || angle == MMRobot.getInstance().mmSystems.intakeEndUnitRotator.getTargetPosition()) {
                    LLResult result = limelight.getLatestResult();

                    if (result != null && result.isValid()) {

                        List<LLResultTypes.DetectorResult> allDetectorResults = result.getDetectorResults();
                        LLResultTypes.DetectorResult dr = allDetectorResults.get(0);
//                          //Rotate claw, then rotate robot to sample, and then open linear intake
//                        double angle = rotateClawToSample(limelight, dr);
                        changeOriention(limelight);
                        openLinearToSample(dr);
                    }
//                    }

                }, this); // do set position void or else wont work}
    }


    public void openLinearToSample(LLResultTypes.DetectorResult result) {
        double distanceFromLimelight = result.getTargetYDegrees();
        if (distanceFromLimelight > maxOpeningLinearCM * LinearIntake.maxOpening){
            distanceFromLimelight = maxOpeningLinearCM * LinearIntake.maxOpening;
        }
        double distance = calculateDistance(distanceFromLimelight) - armLength;
        double distanceInServoDegrees = distance / 130;

        MMRobot.getInstance().mmSystems.linearIntake.setPositionVoid(distanceInServoDegrees);
        MMRobot.getInstance().mmSystems.intakeArm.setPositionVoid(0.55);

        MMRobot.getInstance().mmSystems.telemetry.addData("distance servo -  ", distanceInServoDegrees);
        MMRobot.getInstance().mmSystems.telemetry.addData("distance -  ", distance);
    }

    public Double calculate_distance_vectors(List<Double> vector1,List<Double> vector2){
        return Math.sqrt((vector1.get(0) - vector2.get(0)) * (vector1.get(0) - vector2.get(0)) + (vector1.get(1) - vector2.get(1)) * (vector1.get(1) - vector2.get(1)));
    }

    public double rotateClawToSample(Limelight3A limelight ,LLResultTypes.DetectorResult result){
        double angle = getAngle(limelight, result);
        angle = angle + 90;
        double angleInServoDegrees = angle / 270;
        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPositionVoid(angleInServoDegrees);
        return angleInServoDegrees;
    }

    public double getAngle(Limelight3A limelight , LLResultTypes.DetectorResult result){
        List<List<Double>> corners =  result.getTargetCorners();
        List<Double> cornerUpLeft = corners.get(0);
        List<Double> cornerUpRight = corners.get(1);
        List<Double> cornerDownLeft = corners.get(3);
        Double height  = calculate_distance_vectors(cornerUpLeft, cornerDownLeft) * 1.5;
        Double width  = calculate_distance_vectors(cornerUpLeft, cornerUpRight) * 1.5;
        limelight.pipelineSwitch(1);
        //crop_x, crop_y, crop_width, crop_height = llrobot[0:4] first 4 to send


        double[] inputsPython = {cornerUpLeft.get(0) -50,cornerUpLeft.get(1) -50, width, height};
        limelight.updatePythonInputs(inputsPython);
        double angle = oldAngle;
        double cropped = 0;

        while (angle==oldAngle) {
            double[] outputPython = limelight.getLatestResult().getPythonOutput();
            angle = outputPython[0];
            cropped = outputPython[1];
        }
//        MMRobot.getInstance().mmSystems.telemetry.addData("width - ",width);
//        MMRobot.getInstance().mmSystems.telemetry.addData("height -  ",height);
//        MMRobot.getInstance().mmSystems.telemetry.addData("X left up-  ",cornerUpLeft.get(0));
//        MMRobot.getInstance().mmSystems.telemetry.addData("Y left up -  ",cornerUpLeft.get(1));
//        MMRobot.getInstance().mmSystems.telemetry.addData("did it cropped - ",cropped);
        MMRobot.getInstance().mmSystems.telemetry.addData("angle - ",angle);
        limelight.pipelineSwitch(0);
        return angle;
    }
}