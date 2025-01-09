package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.utils.OpModeType;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.opencv.core.Mat;

import java.util.List;

@TeleOp(name = "Sensor: Limelight3A", group = "Sensor")
public class LimeLight extends MMOpMode {
    private Limelight3A limelight;

    public final double maxOpeningLinearCM = 34.5 ;//cm
    public final double heightFromGround = 11.9; //cm
    public final double angleFixed = 0; //degrees
    public final double sampleHeight = 3.9; //cm


    public LimeLight() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {
        MMRobot.getInstance().mmSystems.initRobotSystems();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();
    }

    public double calculateDistance(double angleFromLimelight){
        double height = heightFromGround - sampleHeight;
        double angle = Math.abs( angleFromLimelight + angleFixed);
        double distance = height / Math.tan(Math.toRadians(angle));
        return distance;
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.controlHub.pullBulkData();

        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            // Access general information
            Pose3D botpose = result.getBotpose();

            if (result.isValid()) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());
                telemetry.addData("Botpose", botpose.toString());


                // Access detector results
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    telemetry.addData("distance: %d", calculateDistance(result.getTy()));
                    MMRobot.getInstance().mmSystems.driveTrain.setHeading(result.getTy());
                    MMRobot.getInstance().mmSystems.linearIntake.setPosition(calculateDistance(result.getTy() / maxOpeningLinearCM * LinearIntake.maxOpening));
                }
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }

        telemetry.update();
    }
}