package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.wpi.first.math.MathUtil;
import lombok.Getter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter;
import org.firstinspires.ftc.teamcode.utils.MathTools;

import java.util.ArrayList;
import java.util.List;

@Config
public class Vision extends SubsystemBase {
    private final Limelight3A camera;
    @Getter
    private LLResult result;

    private LLResult previousResult;

    public static int color; //current color need to be detected
    public static int currentPipeline; // current color


    //detection parameters for distance and strafe:
    public static double CAMERA_HEIGHT = 445;
    public static double CAMERA_ANGLE = 90 - 35.0;
    public static double TARGET_HEIGHT = 39;
    public static double SPECIMEN_HEIGHT = 247.5;

    //debug parameters:
    public static double lastAngle = 0;
    public static double pipelineSwitchFail = 0;
    public static double angleFail = 0;
    private boolean isDataOld = false;

    //size and location of detected sample:
    public static double length = -1;
    public static double height = -1;
    public static double x = -1;
    public static double y = -1;
    public static List<Double> targetLeftUp;

    //telemtry idk man:
    Telemetry telemetry;


    public Vision(final HardwareMap hardwareMap, Telemetry telemetry) {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        camera.pipelineSwitch(1);
        initializeCamera();
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        currentPipeline = 0;
        targetLeftUp = new ArrayList<>();
        targetLeftUp.add(0, 0.0);
        targetLeftUp.add(0, 0.0);
    }

    public void initializeCamera() {
        camera.setPollRateHz(100);
        camera.start();
    }

    //Get degrees in X axis
    public double getTx(double defaultValue) {
        if (result == null) {
            return defaultValue;
        }
        return result.getTx();
    }

    //Get degrees in X axis getting result
    public double getTx(LLResult lastResult) {
        if (lastResult == null) {
            return 0;
        }
        return lastResult.getTx();
    }

    //Get degrees in Y axis
    public double getTy(double defaultValue) {
        if (result == null) {
            return defaultValue;
        }
        return result.getTy();
    }

    //Get degrees in Y axis getting result
    public double getTy(LLResult lastResult) {
        if (lastResult == null) {
            return 0;
        }
        return lastResult.getTy();
    }

    //Check if target is visible
    public boolean isTargetVisible() {
        if (result == null) {
            return false;
        }
        return !MathUtil.isNear(0, result.getTa(), 0.0001);
    }

    //Get distance in Y axis
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

    //Get distance in Y axis with given result
    public Double getDistance(LLResult lastResult) {
        double ty = getTy(lastResult);
        if (ty == 0) {
            return 0.0;
        }
        double angleToGoalDegrees = CAMERA_ANGLE - ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceMM = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
        return Math.abs(distanceMM);
    }

    //Get distance in Y axis but for specimens
    public Double getDistanceSpecimen() {
        double ty = getTy(0.0);
        if (ty == 0) {
            return null;
        }
        double angleToGoalDegrees = CAMERA_ANGLE - ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceMM = (SPECIMEN_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
        return Math.abs(distanceMM);
    }

    // Get the distance for the strafe
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

    // Get the distance for the strafe with given result
    public double getStrafeOffset(LLResult lastResult) {
        if(lastResult != null){}//TODO make it not crash
        double tx = lastResult.getTx();
        if (tx != 0) {
            double tanTX = Math.tan(Math.toRadians(tx));
            double height = CAMERA_HEIGHT - TARGET_HEIGHT;
            double distanceY = getDistance(lastResult);
            double diagonalLength = Math.sqrt(height * height + distanceY * distanceY);
            return tanTX * diagonalLength / 2.54 / 10;
        }
        return 0;
    }

    //Get angle of a sample in servo degrees
    public Double getTurnServoDegree() {
        camera.updatePythonInputs(
                new double[]{0.0, 0.0, 0.0, length, height, x, y, 0.0}
        );
        result = camera.getLatestResult();

        if (result == null) {
            angleFail += 1;
            return null;
        }

        lastAngle = result.getPythonOutput()[0];
        return lastAngle;
    }

    public void trackRed(){
        color = 0;
    }

    public void trackYellow(){
        color = 1;
    }

    public void trackBlue(){
        color = 2;
    }

    public boolean trackSpecimen() {
        currentPipeline = color + 5;
        if (!camera.pipelineSwitch(currentPipeline)) {
            telemetry.addData("failed to switch to python", color);
            pipelineSwitchFail += 1;
            return false;
        }
        return true;
    }

    //Switch to python based detection pipepline
    public boolean switchToPython() {
        currentPipeline = color + 3;
        if (!camera.pipelineSwitch(currentPipeline)) {
            telemetry.addData("failed to switch to python", color);
            pipelineSwitchFail += 1;
            return false;
        }
        return true;
    }

    //Switch to neural-detector based detection pipepline (AI omg ooga booga big words I love man)
    public boolean switchToDetector() {
        FtcDashboard.getInstance().getTelemetry().addData("time sinceupdate",camera.getTimeSinceLastUpdate());
        currentPipeline = color;
        if (!camera.pipelineSwitch(currentPipeline)) {
            telemetry.addData("failed to switch to detector", 0);
            pipelineSwitchFail += 1;
            return false;
        }
        return true;
    }

    //find the closest sample to the middle of the robot
    public void findClosestForPython() {
        if (previousResult != null) {
            List<LLResultTypes.DetectorResult> detectorResults = previousResult.getDetectorResults();
            if (!detectorResults.isEmpty()){
                LLResultTypes.DetectorResult dr = detectorResults.get(0);
                List<List<Double>> corners = dr.getTargetCorners();
                List<Double> leftUp = corners.get(0);
                List<Double> rightUp = corners.get(1);
                List<Double> rightDown = corners.get(2);
                length = MathTools.distance(leftUp, rightUp);
                height = MathTools.distance(rightDown, rightUp);
                targetLeftUp = leftUp;
                x = targetLeftUp.get(0);
                y = targetLeftUp.get(1);
            }
        }
    }

    //like it really means what it says. get the stupid index..
    public double getPipelineIndex() {
        return camera.getStatus().getPipelineIndex();
    }

    //Only change the angle of the intake rotator
    public SequentialCommandGroup angleChange() {
        return new SequentialCommandGroup(
                new InstantCommand(()-> FtcDashboard.getInstance().getTelemetry().addData("angle endered", "enderd")),
                new InstantCommand(()-> FtcDashboard.getInstance().getTelemetry().addData("angle endered2", "not enderd")),
                new InstantCommand(()-> FtcDashboard.getInstance().getTelemetry().addData("angle endered3", "not enderd")),
                new InstantCommand(()-> FtcDashboard.getInstance().getTelemetry().addData("angle endered4", "not enderd")),

                new InstantCommand(() -> findClosestForPython()),

                new InstantCommand(()-> FtcDashboard.getInstance().getTelemetry().addData("angle endered2", "enderd")),

                new WaitUntilCommand(() -> switchToPython()),

                new InstantCommand(()-> FtcDashboard.getInstance().getTelemetry().addData("angle endered3", "enderd")),

                new WaitUntilCommand(() -> camera.getStatus().getPipelineIndex() == Vision.currentPipeline),

                new InstantCommand(()-> FtcDashboard.getInstance().getTelemetry().addData("angle endered4", "enderd")),

                new InstantCommand(() -> camera.updatePythonInputs(new double[]{0.0, 0, 0, length, height, x, y, 0.0})),
                new WaitUntilCommand(() -> camera.getLatestResult().getPythonOutput()[0] != 0),
                limelightGetter.getRotateToSample());
    }

    //use this when you want to save the result instead of switching pipelines like stupid fuck
    public void setPreviousResult() {
        previousResult = camera.getLatestResult();
    }

    //get the previous result
    public LLResult getPreviousResult() {
        return previousResult;
    }

    //Doing every moment, it updates the python inputs, and then updates the result to the latest and freshest one. and telemtry, a lot of telemtry.
    @Override
    public void periodic() {
        //updating the python endlessly
        camera.updatePythonInputs(
                new double[]{0.0, 0.0, 0.0, length, height, x, y, 0.0}
        );

        result = camera.getLatestResult();

        if (result != null) { //if it detects something
            if (currentPipeline == 0) {
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                if (!detectorResults.isEmpty()) {
//                    LLResultTypes.DetectorResult sample = detectorResults.get(0);
//                    List<List<Double>> corners = sample.getTargetCorners();
//                    List<Double> leftUp = corners.get(0);
//                    List<Double> rightUp = corners.get(1);
//                    List<Double> rightDown = corners.get(2);
//                    List<Double> leftDown = corners.get(3);
//                    telemetry.addData("leftUp ->", leftUp);
//                    telemetry.addData("rightUp ->", rightUp);
//                    telemetry.addData("rightDown ->", rightDown);
//                    telemetry.addData("leftDown ->", leftDown);
//                    telemetry.addData("length ->", MathTools.distance(leftUp, rightUp));
//                    telemetry.addData("height ->", MathTools.distance(rightDown, rightUp));
                } else {
                    telemetry.addData("not found anything", -1);
                }
            } else {
                telemetry.addData("Turn Servo Degrees", getTurnServoDegree());
            }
            long staleness = result.getStaleness();

            // Less than 100 milliseconds old
            isDataOld = staleness >= 100;

            telemetry.addData("last width", length);
            telemetry.addData("last height", height);
            telemetry.addData("last x", x);
            telemetry.addData("last y", y);
            telemetry.addData("Tx", result.getTx());
            telemetry.addData("Ty", result.getTy());
            telemetry.addData("Ta", result.getTa());
            telemetry.addData("Strafe Offset", getStrafeOffset());
            telemetry.addData("Distance", getDistance());
            telemetry.addData("angle fail", angleFail);
            telemetry.addData("pipeline fail", pipelineSwitchFail);
            telemetry.addData("last Angle - ", lastAngle);
            telemetry.addData("is data old - ", isDataOld);
            telemetry.addData("pipline", camera.getStatus().getPipelineIndex());
            telemetry.addData("pipline type", camera.getStatus().getPipelineType());
        }
    }
}