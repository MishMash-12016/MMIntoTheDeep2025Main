package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.List;

public class rotateToSample extends CommandBase {
    Limelight3A limelight;

    LLResult result;
    int noResultCounter;


    public double oldAngle;

    public rotateToSample(Limelight3A limelight) {
        this.limelight = limelight;
        addRequirements(
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator);
    }

    @Override
    public void initialize() {
        noResultCounter = 0;
        result = null;
        oldAngle = 0;
    }

    public Double calculate_distance_vectors(List<Double> vector1, List<Double> vector2) {
        return Math.sqrt((vector1.get(0) - vector2.get(0)) * (vector1.get(0) - vector2.get(0)) + (vector1.get(1) - vector2.get(1)) * (vector1.get(1) - vector2.get(1)));
    }

    public double getAngle(Limelight3A limelight) {

        //crop_x, crop_y, crop_width, crop_height = llrobot[0:4] first 4 to send
        double angle = oldAngle;

        while (angle == oldAngle) {
            long startTime = System.currentTimeMillis();
            double[] outputPython = limelight.getLatestResult().getPythonOutput();
            angle = outputPython[0];
            MMRobot.getInstance().mmSystems.telemetry.addData("looking for angle, loop time", System.currentTimeMillis());
            MMRobot.getInstance().mmSystems.telemetry.update();
        }
        //        MMRobot.getInstance().mmSystems.telemetry.addData("width - ",width);
        //        MMRobot.getInstance().mmSystems.telemetry.addData("height -  ",height);
        //        MMRobot.getInstance().mmSystems.telemetry.addData("X left up-  ",cornerUpLeft.get(0));
        //        MMRobot.getInstance().mmSystems.telemetry.addData("Y left up -  ",cornerUpLeft.get(1));
        //        MMRobot.getInstance().mmSystems.telemetry.addData("did it cropped - ",cropped);

        MMRobot.getInstance().mmSystems.telemetry.addData("angle - ", angle);
        return angle;
    }

    @Override
    public void execute() {
        result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> allDetectorResults = result.getDetectorResults();
            LLResultTypes.DetectorResult dr = allDetectorResults.get(0);

            limelight.pipelineSwitch(1);
            double angle = getAngle(limelight);
            limelight.pipelineSwitch(0);

            angle = angle + 90;
            double angleInServoDegrees = angle / 270;

            MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPositionVoid(angleInServoDegrees);
        } else {
            noResultCounter++;
        }
    }

    @Override
    public boolean isFinished() {
        return noResultCounter > 5 || result != null;
    }
}
