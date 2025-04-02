package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.Libraries.exterpolation.ExterpolationMap;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.MathTools;

import java.util.List;

public class openLinearToSample extends CommandBase {

    public boolean finished = false;
    double noResultCounter;

    private double correctionDist = -20; //TODO: change to 20
    //0.06
    //nigvsdghsdgkljdghfshkjdghkjdsgjhksdghjkdsg
    ExterpolationMap exterpolationMap = new ExterpolationMap()
            .put(245, 0.25)
            .put(265, 0.267)
            .put(285, 0.285)
            .put(305, 0.3)
            .put(325, 0.32)
            .put(345, 0.335)
            .put(365, 0.36)
            .put(385, 0.38)
            .put(405, 0.41)
            .put(425, 0.44)
            .put(435, 0.46);



    public openLinearToSample() {
        addRequirements(
                MMRobot.getInstance().mmSystems.linearIntake
        );
    }

    @Override
    public void initialize() {
        noResultCounter = 0;
        finished = false;
    }

    @Override
    public void execute() {
        Double distance = MMRobot.getInstance().mmSystems.vision.getDistance(MMRobot.getInstance().mmSystems.vision.findClosestForDetector());
        if (distance != null) {
            distance += correctionDist;
            double distanceInServoDegrees = exterpolationMap.exterpolate(distance);

            MMRobot.getInstance().mmSystems.linearIntake.setPositionVoid(distanceInServoDegrees);
            MMRobot.getInstance().mmSystems.servoDegrees = distanceInServoDegrees;
            MMRobot.getInstance().mmSystems.telemetry.addData("distance servo -  ", distanceInServoDegrees);
            FtcDashboard.getInstance().getTelemetry().addData("distance servo -  ", distanceInServoDegrees);
            MMRobot.getInstance().mmSystems.telemetry.addData("distance -  ", distance);

            finished = true;
        } else {
            noResultCounter += 1;
        }
    }

    @Override
    public boolean isFinished() {
        return finished || noResultCounter == 5;
    }

//    public static double calculateDistance(double angleFromLimelight) {
//        double height = heightFromGround - sampleHeight;
//        double angle = Math.abs(angleFromLimelight + angleFixed);
//        return height / Math.tan(Math.toRadians(angle));
//    }
}