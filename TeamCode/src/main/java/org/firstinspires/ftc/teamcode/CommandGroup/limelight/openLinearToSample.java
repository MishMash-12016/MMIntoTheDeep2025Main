package org.firstinspires.ftc.teamcode.CommandGroup.limelight;

import static org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter.maxOpeningLinearMM;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Libraries.exterpolation.ExterpolationMap;
import org.firstinspires.ftc.teamcode.MMRobot;

public class openLinearToSample extends CommandBase {

    boolean finished = false;
    double noResultCounter;

    ExterpolationMap exterpolationMap = new ExterpolationMap()
            .put(73.8, 0.19)
            .put(93.3,0.23)
            .put(110,0.247)
            .put(130.6,0.2588)
            .put(160,0.309);

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