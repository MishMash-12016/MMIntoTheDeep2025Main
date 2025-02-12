package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import static org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter.angleFixed;
import static org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter.armLength;
import static org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter.heightFromGround;
import static org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter.maxOpeningLinearCM;
import static org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter.sampleHeight;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;

import java.util.List;

public class openLinearToSample extends CommandBase {
    Limelight3A limelight;

    LLResult result;
    int noResultCounter;

    public openLinearToSample(Limelight3A limelight) {
        this.limelight = limelight;
        addRequirements(
                MMRobot.getInstance().mmSystems.linearIntake,
                MMRobot.getInstance().mmSystems.intakeArm);
    }


    @Override
    public void initialize() {
        noResultCounter = 0;
        result = null;
    }

    @Override
    public void execute() {
        result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> allDetectorResults = result.getDetectorResults();
            LLResultTypes.DetectorResult dr = allDetectorResults.get(0);

            double distanceFromLimelight = dr.getTargetYDegrees();
            double distance = calculateDistance(distanceFromLimelight) - armLength;

            if (distance > maxOpeningLinearCM * LinearIntake.maxOpening) {
                distance = maxOpeningLinearCM * LinearIntake.maxOpening;
            }
            double distanceInServoDegrees = distance / 130;

            MMRobot.getInstance().mmSystems.linearIntake.setPositionVoid(distanceInServoDegrees);
            MMRobot.getInstance().mmSystems.intakeArm.setPositionVoid(0.55);

            MMRobot.getInstance().mmSystems.telemetry.addData("distance servo -  ", distanceInServoDegrees);
            MMRobot.getInstance().mmSystems.telemetry.addData("distance -  ", distance);
        } else {
            noResultCounter++;
        }
    }

    @Override
    public boolean isFinished() {
        return noResultCounter > 5 || result != null;
    }


    public static double calculateDistance(double angleFromLimelight) {
        double height = heightFromGround - sampleHeight;
        double angle = Math.abs(angleFromLimelight + angleFixed);
        return height / Math.tan(Math.toRadians(angle));
    }

}
