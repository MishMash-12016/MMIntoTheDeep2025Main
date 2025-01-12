package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.LimeLight;
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
public class LimeLightTeleOp extends MMOpMode {
    private Limelight3A limelight;

    public final double maxOpeningLinearCM = 34.5 ;//cm
    public final double heightFromGround = 11.9; //cm
    public final double angleFixed = 0; //degrees
    public final double sampleHeight = 3.9; //cm


    public LimeLightTeleOp() {
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
        LLResult result = limelight.getLatestResult();

        List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
        MMRobot.getInstance().mmSystems.limeLight.gotoSample(limelight);

        telemetry.update();
    }
}
