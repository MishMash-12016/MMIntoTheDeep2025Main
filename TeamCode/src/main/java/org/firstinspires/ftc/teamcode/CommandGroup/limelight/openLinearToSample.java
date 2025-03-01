package org.firstinspires.ftc.teamcode.CommandGroup.limelight;

import static org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter.maxOpeningLinearMM;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Libraries.exterpolation.ExterpolationMap;
import org.firstinspires.ftc.teamcode.MMRobot;

public class openLinearToSample extends CommandBase {

    boolean finished = false;
    double noResultCounter;

    ExterpolationMap exterpolationMap = new ExterpolationMap()
            .put(0.89, 0.2)
            .put(110,0.23)
            .put(115, 0.24)
            .put(130, 0.25)
            .put(142.5,0.312)
            .put(160, 0.33)
            .put(180,0.35)
            .put(200, 0.365)
            .put(218,0.38)
            .put(247,0.447);

    public  openLinearToSample() {
        addRequirements(
                MMRobot.getInstance().mmSystems.linearIntake,
                MMRobot.getInstance().mmSystems.intakeArm
        );
    }

    @Override
    public void initialize() {
        noResultCounter = 0;
        finished = false;
    }

    @Override
    public void execute() {
        Double distance = MMRobot.getInstance().mmSystems.vision.getDistance();
        if (distance!=null){

            double distanceInServoDegrees = exterpolationMap.exterpolate(distance);
//            distanceInServoDegrees *= 0.47;


            MMRobot.getInstance().mmSystems.linearIntake.setPositionVoid(distanceInServoDegrees);
            MMRobot.getInstance().mmSystems.intakeArm.setPositionVoid(0.57);

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
    public boolean isFinished() {
        return finished || noResultCounter == 5;
    }

//    public static double calculateDistance(double angleFromLimelight) {
//        double height = heightFromGround - sampleHeight;
//        double angle = Math.abs(angleFromLimelight + angleFixed);
//        return height / Math.tan(Math.toRadians(angle));
//    }
}