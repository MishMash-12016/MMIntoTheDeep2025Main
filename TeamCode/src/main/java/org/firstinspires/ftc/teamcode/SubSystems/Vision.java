package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.wpi.first.math.MathUtil;
import lombok.Getter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.MathTools;

import java.util.ArrayList;
import java.util.List;

@Config
public class Vision extends SubsystemBase {
    private final Limelight3A camera;

    private boolean isDataOld = false;
    @Getter
    private LLResult result;


    public static double CAMERA_HEIGHT = 445;
    public static double CAMERA_ANGLE = 90 - 35.0;
    public static double TARGET_HEIGHT = 39;
    public static double SPECIMEN_HEIGHT = 247.5;
    public static double cameraStrafeToBot = 5;

    public static double armLength = 0;

    public static double lastAngle = 0;
    public static int currentPipeline;
    public static double pipelineSwitchFail = 0;
    public static double angleFail = 0;
    public static double length = -1;
    public static double height = -1;
    public static double x = -1;
    public static double y = -1;
    public static List<Double> targetLeftUp;


    public static double lengthForDetector = -1;
    public static double heightForDetector = -1;
    public static double xForDetector = -1;
    public static double yForDetector = -1;
    public static List<Double> targetLeftUpForDetector;
    public static LLResultTypes.DetectorResult detectorResultForDetector;
    public static double linearPointX = 226;
    Telemetry telemetry;


    public Vision(final HardwareMap hardwareMap, Telemetry telemetry) {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        camera.pipelineSwitch(0);
//        led = hardwareMap.get(Servo.class, "LED");
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initializeCamera();
        currentPipeline = 0;
        targetLeftUp = new ArrayList<>();
        targetLeftUp.add(0, 0.0);
        targetLeftUp.add(0, 0.0);
    }

//    public void setLEDPWM() {
//        led.setPosition(ledPWM);
//    }

    public void initializeCamera() {
        camera.setPollRateHz(100);
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

    public double getTy(LLResultTypes.DetectorResult dr, double defaultValue) {
        if (dr == null) {
            return defaultValue;
        }
        return dr.getTargetYDegrees();
    }

    public boolean isTargetVisible() {
        if (result == null) {
            return false;
        }
        return !MathUtil.isNear(0, result.getTa(), 0.0001);
    }

    public Double getDistance() {
        double ty = getTy(0.0);
        if (ty == 0) {
            return 0.0;
        }
        double angleToGoalDegrees = CAMERA_ANGLE - ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceMM = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
        return Math.abs(distanceMM);
    }

    public Double getDistance(LLResultTypes.DetectorResult dr) {
        double ty = getTy(dr, 0.0);
        if (ty == 0) {
            return null;
        }
        double angleToGoalDegrees = CAMERA_ANGLE - ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceMM = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
        return Math.abs(distanceMM) - armLength;
    }

    public Double getDistanceSpecimen() {
        double ty = getTy(0.0);
        if (ty == 0) {
            return null;
        }
        double angleToGoalDegrees = CAMERA_ANGLE - ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceMM = (SPECIMEN_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
        return Math.abs(distanceMM) - armLength;
    }

    // Get the strafe
    public double getStrafeOffset() {
        double tx = getTx(0);
        if (tx != 0) {
            double tanTX = Math.tan(Math.toRadians(tx));
            double height = CAMERA_HEIGHT - TARGET_HEIGHT;
            double distanceY = getDistance();
            double diagonalLength = Math.sqrt(height * height + distanceY * distanceY);
            return tanTX * diagonalLength / 2.54 / 10;
        }
        return 0;
    }

    public Double getTurnServoDegree() {

        result = camera.getLatestResult();

        if (result == null) {
            angleFail += 1;
            return null;
        }

        lastAngle = result.getPythonOutput()[0];
        return lastAngle;
    }

    public boolean trackRedPython() {
        currentPipeline = 1;
        if (!camera.pipelineSwitch(currentPipeline)) {
            telemetry.addData("failed to switch to red", 0);
            pipelineSwitchFail += 1;
            return false;
        }
        return true;
    }

    public void trackRedDetector() {
        currentPipeline = 0;
        if (!camera.pipelineSwitch(currentPipeline)) {
            telemetry.addData("failed to switch to red", 0);
            pipelineSwitchFail += 1;
        }
    }

    public void findClosestForPython() {
        result = camera.getLatestResult();
        length = 0;
        height = 0;
        x = 1000000;
        y = 1000000;
        if (result != null) {
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
            for (LLResultTypes.DetectorResult dr : detectorResults) {
                List<List<Double>> corners = dr.getTargetCorners();
                List<Double> leftUp = corners.get(0);
                List<Double> rightUp = corners.get(1);
                List<Double> rightDown = corners.get(2);
                if (Math.abs(linearPointX - (leftUp.get(0) + MathTools.distance(leftUp, rightUp) / 2)) < Math.abs(linearPointX - (x + length / 2))) {
                    length = MathTools.distance(leftUp, rightUp);
                    height = MathTools.distance(rightDown, rightUp);
                    targetLeftUp = leftUp;
                    x = targetLeftUp.get(0);
                    y = targetLeftUp.get(1);
                }
            }
        }
    }

    public LLResultTypes.DetectorResult findClosestForDetector() {
        result = camera.getLatestResult();
        lengthForDetector = 0;
        heightForDetector = 0;
        xForDetector = 1000000;
        yForDetector = 1000000;

        List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
        for (LLResultTypes.DetectorResult dr : detectorResults) {
            List<List<Double>> corners = dr.getTargetCorners();
            List<Double> leftUp = corners.get(0);
            List<Double> rightUp = corners.get(1);
            List<Double> rightDown = corners.get(2);
            if (Math.abs(linearPointX - (leftUp.get(0) + MathTools.distance(leftUp, rightUp) / 2)) < Math.abs(linearPointX - (xForDetector + lengthForDetector / 2))) {
                lengthForDetector = MathTools.distance(leftUp, rightUp);
                heightForDetector = MathTools.distance(rightDown, rightUp);
                targetLeftUpForDetector = leftUp;
                xForDetector = targetLeftUpForDetector.get(0);
                yForDetector = targetLeftUpForDetector.get(1);
                detectorResultForDetector = dr;
            }
        }
        return detectorResultForDetector;
    }

    public double getPipelineIndex(){
        return camera.getStatus().getPipelineIndex();
    }

    public SequentialCommandGroup onlyAngleChange() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> findClosestForPython()),
                new InstantCommand(() -> trackRedPython()),
                new WaitUntilCommand(() -> camera.getStatus().getPipelineIndex() == 1),
                new InstantCommand(() -> camera.updatePythonInputs(new double[]{0.0, 0, 0, length, height, x, y, 0.0})),
                new WaitUntilCommand(()->camera.getLatestResult().getPythonOutput()[0]!=0),
                limelightGetter.getRotateToSample());
    }


    @Override
    public void periodic() {
        //updating the python endlessly
        camera.updatePythonInputs(
                new double[] { 0.0, 0.0, 0.0, length, height, x, y, 0.0 }
        );

        result = camera.getLatestResult();

        if (result != null) { //if it detects something
            if (currentPipeline == 0) {
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                if (!detectorResults.isEmpty()) {
                    LLResultTypes.DetectorResult sample = detectorResults.get(0);
                    List<List<Double>> corners = sample.getTargetCorners();
                    List<Double> leftUp = corners.get(0);
                    List<Double> rightUp = corners.get(1);
                    List<Double> rightDown = corners.get(2);
                    List<Double> leftDown = corners.get(3);
                    telemetry.addData("leftUp ->", leftUp);
                    telemetry.addData("rightUp ->", rightUp);
                    telemetry.addData("rightDown ->", rightDown);
                    telemetry.addData("leftDown ->", leftDown);
                    telemetry.addData("length ->", MathTools.distance(leftUp, rightUp));
                    telemetry.addData("height ->", MathTools.distance(rightDown, rightUp));
                } else {
                    telemetry.addData("not found anything", -1);
                }
            } else {
                telemetry.addData("Turn Servo Degrees", getTurnServoDegree());
            }
            long staleness = result.getStaleness();

            // Less than 100 milliseconds old
            isDataOld = staleness >= 100;

            telemetry.addData("width", length);
            telemetry.addData("height", height);
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("Tx", result.getTx());
            telemetry.addData("Ty", result.getTy());
            telemetry.addData("Ta", result.getTa());
            telemetry.addData("Strafe Offset", getStrafeOffset());
            telemetry.addData("Distance", getDistance());
            telemetry.addData("angle fail", angleFail);
            telemetry.addData("pipeline fail", pipelineSwitchFail);
            telemetry.addData("last Angle - ", lastAngle);
            telemetry.addData("pipline", camera.getStatus().getPipelineIndex());
            telemetry.addData("pipline type", camera.getStatus().getPipelineType());
        }
    }
}