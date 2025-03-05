package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;

import java.util.List;

public class rotateToSample extends CommandBase {
    public double oldAngle;
    boolean finished = false;
    double noResultCounter;

    public rotateToSample() {
        addRequirements(
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator
        );
    }

    @Override
    public void initialize() {
        oldAngle = 0;
        noResultCounter = 0;
        finished = false;
        MMRobot.getInstance().mmSystems.vision.startTracking();
    }


    @Override
    public void execute() {
        Double angle = MMRobot.getInstance().mmSystems.vision.getTurnServoDegree();
        if (angle != null) {
            angle = MMRobot.getInstance().mmSystems.vision.getTurnServoDegree();
            if (angle>=0 && angle<= 90){
                angle /= 270;
                angle = IntakeEndUnitRotator.IntakeRotatorState.INTAKE_SAMPLE_POSE.position.get() - angle;
            }
            else{
                angle = 180 - angle;
                angle /= 270;
                angle = IntakeEndUnitRotator.IntakeRotatorState.INTAKE_SAMPLE_POSE.position.get() + angle;
            }
//            angle += 90;
//            double angleInServoDegrees = angle / 270;
//        MMRobot.getInstance().mmSystems.telemetry.addData("found the stupid sample", 0);
            MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPositionVoid(angle);
            MMRobot.getInstance().mmSystems.telemetry.update();
            finished = true;
        } else
            noResultCounter++;
    }

    @Override
    public void end(boolean interrupted) {
        MMRobot.getInstance().mmSystems.vision.stopTracking();
    }

    @Override
    public boolean isFinished() {
        return finished || noResultCounter == 5;
    }
}
