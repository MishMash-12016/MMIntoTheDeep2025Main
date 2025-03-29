package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.wpi.first.math.MathUtil;
import lombok.Getter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.MathTools;

import java.util.ArrayList;
import java.util.List;

@Config
public class Vision extends SubsystemBase {
    private final Limelight3A camera;

    private double trackSample = 0;
    private double autoOrTele = 0;

//    private final Servo led;


    @Getter private boolean isDataOld = false;
    @Getter private LLResult result;


    public static double CAMERA_HEIGHT = 424;
    public static double CAMERA_ANGLE = -45.0;
    public static double TARGET_HEIGHT = 39;
    public static double SPECIMEN_HEIGHT = 247.5;

    public static double strafeConversionFactor = 1/2.54;
    public static double cameraStrafeToBot = 0.0;

    public static double sampleToRobotDistance = 105;

    public static double opModeType = 0;
    public static int currentPipeline;
    public static double pipelineSwitchFail = 0;
    public static double length = -1;
    public static double height = -1;
    public static double x = -1;
    public static double y = -1;
    public static List<Double> targetLeftUp;
    public static double linearPointX = 200;




    Telemetry telemetry;

    public Vision(final HardwareMap hardwareMap, Telemetry telemetry) {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        camera.pipelineSwitch(0);
//        led = hardwareMap.get(Servo.class, "LED");
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initializeCamera();
        currentPipeline =0;
        targetLeftUp = new ArrayList<>();
        targetLeftUp.add(0,0.0);
        targetLeftUp.add(0,0.0);
    }

//    public void setLEDPWM() {
//        led.setPosition(ledPWM);
//    }

    public void initializeCamera() {
        camera.setPollRateHz(50);
        camera.start();
    }




    public double getTx(double defaultValue) {
        if (result == null) {
            return defaultValue;
        }
        return result.getTx();
    }

    public double getTy(double defaultValue) {
        if (result == null) {
            return defaultValue;
        }
        return result.getTy();
    }

    public boolean isTargetVisible() {
        if (result == null) {
            return false;
        }
        return !MathUtil.isNear(0, result.getTa(), 0.0001);
    }

    public Double getDistance() {
        double ty = getTy(0.0);
        if (ty == 0){
            return null;
        }
        double angleToGoalDegrees = CAMERA_ANGLE + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceMM = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
        return Math.abs(distanceMM) - sampleToRobotDistance - 10;
    }

    public Double getDistanceSpecimen() {
        double ty = getTy(0.0);
        if (ty == 0){
            return null;
        }
        double angleToGoalDegrees = CAMERA_ANGLE + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceMM = (SPECIMEN_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
        return Math.abs(distanceMM) - sampleToRobotDistance - 10;
    }

    // Get the strafe
    public double  getStrafeOffset() {
        double tx = getTx(0);
        if (tx != 0) {
            return tx * strafeConversionFactor - cameraStrafeToBot;
        }
        return 0;
    }

    public Double getTurnServoDegree() {
        if (result == null) {
            return null;
        }
        return result.getPythonOutput()[3];
    }

    public void startTracking(){
        trackSample = 1;
    }

    public void stopTracking(){
        trackSample = 0;
    }

    public void trackYellowPython(){
        currentPipeline = 5;
        if (!camera.pipelineSwitch(currentPipeline)){
            telemetry.addData("failed to switch to yellow", 0);
            pipelineSwitchFail += 1;
        }
    }

    public void trackRedPython(){
        currentPipeline = 4;
        if (!camera.pipelineSwitch(currentPipeline)){
            telemetry.addData("failed to switch to red", 0);
            pipelineSwitchFail += 1;
        }
    }

    public void trackBluePython(){
        currentPipeline = 3;
        if (!camera.pipelineSwitch(currentPipeline)){
            telemetry.addData("failed to switch to blue", 0);
            pipelineSwitchFail += 1;
        }
    }


    public void trackYellowDetector(){
        currentPipeline = 0;
        if (!camera.pipelineSwitch(currentPipeline)){
            telemetry.addData("failed to switch to yellow", 0);
            pipelineSwitchFail += 1;
        }
    }

    public void trackRedDetector(){
        currentPipeline = 1;
        if (!camera.pipelineSwitch(currentPipeline)){
            telemetry.addData("failed to switch to red", 0);
            pipelineSwitchFail += 1;
        }
    }

    public void trackBlueDetector(){
        currentPipeline = 2;
        if (!camera.pipelineSwitch(currentPipeline)){
            telemetry.addData("failed to switch to blue", 0);
            pipelineSwitchFail += 1;
        }
    }
    public void reset(){
        height = -1;
        length = -1;
        x = -1;
        y = -1;
    }

    public void auto(){opModeType = 1;}
    public void teleOp(){opModeType = 0;}

    public void findRightmost(){
        List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
        for (LLResultTypes.DetectorResult dr : detectorResults) {
            List<List<Double>> corners = dr.getTargetCorners();
            List<Double> leftUp = corners.get(0);
            List<Double> rightUp = corners.get(1);
            List<Double> rightDown = corners.get(2);
            List<Double> leftDown = corners.get(3);
            if (Math.abs(linearPointX - (leftUp.get(0) + length / 2)) < Math.abs(linearPointX - (targetLeftUp.get(0) + length / 2))){
                length = MathTools.distance(leftUp, rightUp);
                height = MathTools.distance(rightDown, rightUp);
                targetLeftUp = leftUp;
                x = targetLeftUp.get(0);
                y = targetLeftUp.get(1);
            }
        }
    }


    @Override
    public void periodic() {
        if (currentPipeline != 0){
            //updating the python endlessly
            camera.updatePythonInputs(
                    new double[] {0.0 , 1, opModeType, length, height, x, y, 0.0});
        }
        result = camera.getLatestResult();

        if (result != null) { //if it detects something
            if (currentPipeline == 0){
                List<LLResultTypes.DetectorResult> detectorResults =result.getDetectorResults();
                if (!detectorResults.isEmpty()){
                    LLResultTypes.DetectorResult sample = detectorResults.get(0);
                    List<List<Double>> corners = sample.getTargetCorners();
                    List<Double> leftUp = corners.get(0);
                    List<Double> rightUp = corners.get(1);
                    List<Double> rightDown = corners.get(2);
                    List<Double> leftDown = corners.get(3);
                    length = MathTools.distance(leftUp, rightUp);
                    height = MathTools.distance(rightDown, rightUp);
                    telemetry.addData("leftUp ->", leftUp);
                    telemetry.addData("rightUp ->", rightUp);
                    telemetry.addData("rightDown ->", rightDown);
                    telemetry.addData("leftDown ->", leftDown);
                    telemetry.addData("length ->", length);
                    telemetry.addData("height ->", height);
                }
                else {
                    telemetry.addData("not found anything", -1);
                }

            }
            else {
                telemetry.addData("Strafe Offset", getStrafeOffset());
                telemetry.addData("Distance", getDistance());
                telemetry.addData("Turn Servo Degrees", getTurnServoDegree());
            }
            long staleness = result.getStaleness();

            // Less than 100 milliseconds old
            isDataOld = staleness >= 100;

            telemetry.addData("Tx", result.getTx());
            telemetry.addData("Ty", result.getTy());
            telemetry.addData("Ta", result.getTa());
        }
//        telemetry.update();
    }
}