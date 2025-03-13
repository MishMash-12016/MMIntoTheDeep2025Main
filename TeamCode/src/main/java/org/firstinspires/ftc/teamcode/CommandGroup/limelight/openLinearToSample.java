package org.firstinspires.ftc.teamcode.CommandGroup.limelight;

import static org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter.maxOpeningLinearMM;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Libraries.exterpolation.ExterpolationMap;
import org.firstinspires.ftc.teamcode.MMRobot;

public class openLinearToSample extends CommandBase {

    boolean finished = false;
    double noResultCounter;

    private double correctionDist = 20; //TODO: change to 20

    ExterpolationMap exterpolationMap = new ExterpolationMap()
            .put(63,0.33)
            .put(75,0.356)
            .put(85,0.361)
            .put(95,0.37)
            .put(101,0.38)
            .put(110.4,0.39)
            .put(115.4,0.399)
            .put(130,0.418)
            .put(140,0.423)
            .put(155,0.434)
            .put(160,0.448)
            .put(170,0.455)
            .put(183,0.46)
            .put(191,0.48)
            .put(202,0.49)
            .put(212,0.51)
            .put(218,0.52)
            .put(230,0.53)
            .put(240,0.55)
            .put(250,0.57)
            .put(260,0.596)
            .put(270,0.7);

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