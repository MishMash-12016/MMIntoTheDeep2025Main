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
    public boolean finished;

    public rotateToSample(Limelight3A limelight) {
        this.limelight = limelight;
        addRequirements(
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator
        );
    }

    @Override
    public void initialize() {
        noResultCounter = 0;
        result = null;
        oldAngle = 0;
        finished = false;
    }

    public Double calculate_distance_vectors(List<Double> vector1, List<Double> vector2) {
        return Math.sqrt((vector1.get(0) - vector2.get(0)) * (vector1.get(0) - vector2.get(0)) + (vector1.get(1) - vector2.get(1)) * (vector1.get(1) - vector2.get(1)));
    }

    public double getAngle(Limelight3A limelight) {

        //crop_x, crop_y, crop_width, crop_height = llrobot[0:4] first 4 to send
        double angle = oldAngle;

        while (angle == oldAngle && noResultCounter <= 5) {
            noResultCounter +=1;
            double[] outputPython = limelight.getLatestResult().getPythonOutput();
            angle = outputPython[0];
            MMRobot.getInstance().mmSystems.telemetry.addData("looking for angle, loop time", noResultCounter);
            MMRobot.getInstance().mmSystems.telemetry.update();
        }
        MMRobot.getInstance().mmSystems.telemetry.addData("angle - ", angle);
        MMRobot.getInstance().mmSystems.telemetry.update();
        return angle;
    }

    @Override
    public void execute() {
        limelight.pipelineSwitch(1);
        MMRobot.getInstance().mmSystems.telemetry.addData("start execute",0);
        double angle = getAngle(limelight);

        angle = angle + 90;
        double angleInServoDegrees = angle / 270;
        MMRobot.getInstance().mmSystems.telemetry.addData("found the stupid sample", 0);
        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPositionVoid(angleInServoDegrees);
        finished = true;
        MMRobot.getInstance().mmSystems.telemetry.update();
    }

    @Override
    public boolean isFinished() {
        return finished || noResultCounter == 5;
    }
}
