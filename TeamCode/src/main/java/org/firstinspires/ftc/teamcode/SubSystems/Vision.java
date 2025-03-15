package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import edu.wpi.first.math.MathUtil;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Vision extends SubsystemBase {
    private final Limelight3A camera;

    private double trackSample = 0;

//    private final Servo led;


    @Getter private boolean isDataOld = false;
    @Getter @Setter private SampleColor detectionColor = SampleColor.YELLOW;
    @Getter private LLResult result;


    public static double CAMERA_HEIGHT = 420; //  TODO :420
    public static double CAMERA_ANGLE = -45.0;
    public static double TARGET_HEIGHT = 39;

    public static double strafeConversionFactor = 10;
    public static double cameraStrafeToBot = 0.0;

    public static double sampleToRobotDistance = 105;

    public static double opModeType = 0;


    Telemetry telemetry;

    public Vision(final HardwareMap hardwareMap, Telemetry telemetry) {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        camera.pipelineSwitch(0);
//        led = hardwareMap.get(Servo.class, "LED");
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initializeCamera();
    }

//    public void setLEDPWM() {
//        led.setPosition(ledPWM);
//    }

    public void initializeCamera() {
        camera.setPollRateHz(50);
        camera.start();
    }


    @RequiredArgsConstructor
    public enum SampleColor {
        RED(0.0),
        BLUE(1.0),
        YELLOW(2.0);

        private final double colorVal;
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

    public double getDistance() {
        double ty = getTy(0.0);
        if (ty == 0){
            return 0;
        }
        double angleToGoalDegrees = CAMERA_ANGLE + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceMM = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
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

    public void trackYellow(){
        camera.pipelineSwitch(0);
    }

    public void trackRed(){
        camera.pipelineSwitch(1);
    }

    public void trackBlue(){
        camera.pipelineSwitch(2);
    }

    public void auto(){opModeType = 1;}
    public void teleOp(){opModeType = 0;}


    @Override
    public void periodic() {
        camera.updatePythonInputs(
                new double[] {detectionColor.colorVal, trackSample, opModeType, 0.0, 0.0, 0.0, 0.0, 0.0});
        result = camera.getLatestResult();

        if (result != null) {
            long staleness = result.getStaleness();
            // Less than 100 milliseconds old
            isDataOld = staleness >= 100;
            telemetry.addData("Strafe Offset", getStrafeOffset());
            telemetry.addData("Distance", getDistance());
            telemetry.addData("Turn Servo Degrees", getTurnServoDegree());

                  telemetry.addData("Tx", result.getTx());
                  telemetry.addData("Ty", result.getTy());
                  telemetry.addData("Ta", result.getTa());
             telemetry.update();
        }
    }
}