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
                MMRobot.getInstance().mmSystems.intakeArm
        );
    }

    @Override
    public void initialize() {
        noResultCounter = 0;
        result = null;
        limelight.pipelineSwitch(1);
    }

    @Override
    public void execute() {
        MMRobot.getInstance().mmSystems.telemetry.addData("linear to sample", 0);
        double[] outputPython = limelight.getLatestResult().getPythonOutput();

        double distanceFromLimelight = outputPython[2];;
        double distance = calculateDistance(distanceFromLimelight) - armLength;

        double distanceInServoDegrees = distance / 130;

        if (distanceInServoDegrees > 0.6) {
            distance = 0.6;
            MMRobot.getInstance().mmSystems.telemetry.addData("maxed out so sad -  ", distance);
        }

        MMRobot.getInstance().mmSystems.linearIntake.setPosition(distanceInServoDegrees).initialize();
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(distanceInServoDegrees).execute();
        MMRobot.getInstance().mmSystems.intakeArm.setPosition(0.54).initialize();
        MMRobot.getInstance().mmSystems.intakeArm.setPosition(0.54).execute();

        MMRobot.getInstance().mmSystems.telemetry.addData("distance servo -  ", distanceInServoDegrees);
        MMRobot.getInstance().mmSystems.telemetry.addData("distance -  ", distance);

        MMRobot.getInstance().mmSystems.telemetry.update();
    }

    @Override
    public boolean isFinished() {
        return result != null || noResultCounter == 5;
    }

    public static double calculateDistance(double angleFromLimelight) {
        double height = heightFromGround - sampleHeight;
        double angle = Math.abs(angleFromLimelight + angleFixed);
        return height / Math.tan(Math.toRadians(angle));
    }
}