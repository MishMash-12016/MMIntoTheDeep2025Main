package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.MMRobot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        PIDController pidController = new PIDController(0.017, 0, 0.0005);
        pidController.setSetPoint(0);
        pidController.setTolerance(0.3);

        return new RunCommand(
                () -> {

                    ElapsedTime realElapsedTime = new ElapsedTime();

                    double angle = 0;
//                    while (angle == 0 || angle == MMRobot.getInstance().mmSystems.intakeEndUnitRotator.getTargetPosition()) {
                        LLResult result = limelight.getLatestResult();

                        if (result != null && result.isValid()) {

                            List<LLResultTypes.DetectorResult> allDetectorResults = result.getDetectorResults();
                            LLResultTypes.DetectorResult dr = allDetectorResults.get(0);
//                        //Rotate claw, then rotate robot to sample, and then open linear intake
//                            angle = rotateClawToSample(limelight, dr);
                            MMRobot.getInstance().mmSystems.driveTrain.drive(0, 0, pidController.calculate(-dr.getTargetXDegrees()));
                            MMRobot.getInstance().mmSystems.telemetry.addData("dx - ", -dr.getTargetXDegrees());
//                        openLinearToSample(dr);
                        } else {
                            MMRobot.getInstance().mmSystems.telemetry.addData("not valid ): - ", 0);
                        }
//                    }

//                    MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPositionVoid(angle);

                    MMRobot.getInstance().mmSystems.telemetry.addData("elapse time", realElapsedTime.milliseconds());

//                    long t2 = System.currentTimeMillis(); // Start time
//                    MMRobot.getInstance().mmSystems.telemetry.addData("time all - ",t2 - t1);

                }, this) // do set position void or else wont work
                .interruptOn(()-> pidController.atSetPoint() && limelight.getLatestResult().isValid());
    }

//    public void openLinearToSample(LLResultTypes.DetectorResult result) {
//        MMRobot.getInstance().mmSystems.linearIntake.setPosition(
//                calculateDistance(result.getTargetYDegrees() / (maxOpeningLinearCM * LinearIntake.maxOpening)));
//    }

    public Double calculate_distance_vectors(List<Double> vector1,List<Double> vector2){
        return Math.sqrt((vector1.get(0) - vector2.get(0)) * (vector1.get(0) - vector2.get(0)) + (vector1.get(1) - vector2.get(1)) * (vector1.get(1) - vector2.get(1)));
    }

    public double rotateClawToSample(Limelight3A limelight ,LLResultTypes.DetectorResult result){
        double angle = getAngle(limelight, result);
        angle = angle + 90;
        double angleInServoDegrees = angle / 270;
        MMRobot.getInstance().mmSystems.telemetry.addData("angle for servo= ",angleInServoDegrees);

        return angleInServoDegrees;
    }

    public double getAngle(Limelight3A limelight , LLResultTypes.DetectorResult result){
        List<List<Double>> corners =  result.getTargetCorners();
        List<Double> cornerUpLeft = corners.get(0);
        List<Double> cornerUpRight = corners.get(1);
        List<Double> cornerDownLeft = corners.get(3);
        Double height  = calculate_distance_vectors(cornerUpLeft, cornerDownLeft) * 1.3;
        Double width  = calculate_distance_vectors(cornerUpLeft, cornerUpRight) * 1.3;
        limelight.pipelineSwitch(1);
        //crop_x, crop_y, crop_width, crop_height = llrobot[0:4] first 4 to send


        double[] inputsPython = {cornerUpLeft.get(0) -50,cornerUpLeft.get(1) -50, width, height};
        limelight.updatePythonInputs(inputsPython);
        double[] outputPython = limelight.getLatestResult().getPythonOutput();

        double angle = outputPython[0];
        double cropped = outputPython[1];
//        for (int i =0; i<outputPython.length;i++){
//            MMRobot.getInstance().mmSystems.telemetry.addData("output - ",outputPython[i]);
//        }
        MMRobot.getInstance().mmSystems.telemetry.addData("width - ",width);
        MMRobot.getInstance().mmSystems.telemetry.addData("height -  ",height);
        MMRobot.getInstance().mmSystems.telemetry.addData("X left up-  ",cornerUpLeft.get(0));
        MMRobot.getInstance().mmSystems.telemetry.addData("Y left up -  ",cornerUpLeft.get(1));
        MMRobot.getInstance().mmSystems.telemetry.addData("did it cropped - ",cropped);
        MMRobot.getInstance().mmSystems.telemetry.addData("angle - ",angle);
        limelight.pipelineSwitch(0);
        return angle;
    }
}