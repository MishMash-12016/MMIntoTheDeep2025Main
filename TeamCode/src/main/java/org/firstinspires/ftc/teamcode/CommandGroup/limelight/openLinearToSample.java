package org.firstinspires.ftc.teamcode.CommandGroup.limelight;

import static org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter.maxOpeningLinearMM;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Libraries.exterpolation.ExterpolationMap;
import org.firstinspires.ftc.teamcode.MMRobot;

public class openLinearToSample extends CommandBase {

    boolean finished = false;
    double noResultCounter;

    ExterpolationMap exterpolationMap = new ExterpolationMap()
            .put(63,0.33)
            .put(75,0.356)
            .put(85,0.361)
            .put(95,0.37)
            .put(101,0.38)
            .put(110.4,0.39)
            .put(115.4,0.399)
            .put(140,0.423)
            .put(155,0.434)
            .put(170,0.455)
            .put(202,0.49)
            .put(230,0.53)
            .put(240,0.54)
            .put(246.6,0.56)
            .put(270,0.6);

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
        Double distance = MMRobot.getInstance().mmSystems.vision.getDistance();
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