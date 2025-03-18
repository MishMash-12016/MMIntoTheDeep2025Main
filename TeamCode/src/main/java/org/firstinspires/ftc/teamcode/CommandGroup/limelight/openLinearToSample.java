package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Libraries.exterpolation.ExterpolationMap;
import org.firstinspires.ftc.teamcode.MMRobot;

public class openLinearToSample extends CommandBase {

    boolean finished = false;
    double noResultCounter;

    private double correctionDist = -20; //TODO: change to 20
//0.06
    //nigvsdghsdgkljdghfshkjdghkjdsgjhksdghjkdsg
    ExterpolationMap exterpolationMap = new ExterpolationMap()
            .put(63,0.27)
            .put(75,0.29)
            .put(85,0.3)
            .put(95,0.31)
            .put(101,0.32)
            .put(110.4,0.33)
            .put(115.4,0.339)
            .put(130,0.358)
            .put(140,0.363)
            .put(155,0.374)
            .put(160,0.388)
            .put(170,0.395)
            .put(183,0.4)
            .put(191,0.42)
            .put(202,0.43)
            .put(212,0.45)
            .put(218,0.46)
            .put(230,0.47)
            .put(240,0.48)
            .put(250,0.51)
            .put(260,0.536)
            .put(270,0.64);

    public  openLinearToSample() {
        addRequirements(
                MMRobot.getInstance().mmSystems.linearIntake
        );
    }

    @Override
    public void initialize() {
        noResultCounter = 0;
        finished = false;
        MMRobot.getInstance().mmSystems.vision.startTracking();
    }

    @Override
    public void execute() {
        Double distance = MMRobot.getInstance().mmSystems.vision.getDistance() + correctionDist;
        if (distance!=null){

            double distanceInServoDegrees = exterpolationMap.exterpolate(distance);

            MMRobot.getInstance().mmSystems.linearIntake.setPositionVoid(distanceInServoDegrees);

            MMRobot.getInstance().mmSystems.telemetry.addData("distance servo -  ", distanceInServoDegrees);
            MMRobot.getInstance().mmSystems.telemetry.addData("distance -  ", distance);

            MMRobot.getInstance().mmSystems.telemetry.update();
            finished = true;
        }
        else {
            noResultCounter += 1;
        }
    }

    @Override
    public void end(boolean interrupted) {
        MMRobot.getInstance().mmSystems.vision.stopTracking();
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