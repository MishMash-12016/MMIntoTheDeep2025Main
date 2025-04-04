package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;

import java.util.List;

public class rotateToSample extends CommandBase {
    boolean finished = false;
    double noResultCounter;
    Double angle;

    public rotateToSample() {
        addRequirements(
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator
        );
    }

    @Override
    public void initialize() {
        noResultCounter = 0;
        finished = false;
    }


    @Override
    public void execute() {
        angle = MMRobot.getInstance().mmSystems.vision.getTurnServoDegree();
        if (angle != null) {
            if ((angle <= 5 || angle >=175) || (angle>=85&& angle <= 95)){
                angle = 45.0;
                angle /= 270;
                angle = IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE.position.get() + angle;
            }
            else if (angle>=0 && angle<= 90){
                angle /= 270;
                angle = IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE.position.get() - angle;
            }
            else{
                angle = 180 - angle;
                angle /= 270;
                angle = IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE.position.get() + angle;
            }
            MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPositionVoid(angle);
//            MMRobot.getInstance().mmSystems.telemetry.update();
            finished = true;
        } else
            noResultCounter++;
    }

    @Override
    public void end(boolean interrupted) {
        FtcDashboard.getInstance().getTelemetry().addData("noResult = ",noResultCounter);
        FtcDashboard.getInstance().getTelemetry().addData("angle rotate = ",angle);
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public boolean isFinished() {
//        return finished || noResultCounter == 5;
        return true;
    }
}
