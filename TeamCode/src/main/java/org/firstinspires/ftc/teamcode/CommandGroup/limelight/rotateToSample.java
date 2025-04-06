package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;

import java.util.List;

public class rotateToSample extends InstantCommand {
    Double angle;

    public rotateToSample() {
        addRequirements(
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator
        );
    }

    @Override
    public void initialize() {
        angle = MMRobot.getInstance().mmSystems.vision.getTurnServoDegree();
        if (angle != null) {
            if (angle>=0 && angle<= 90){
                angle /= 270;
                angle = IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE.position.get() - angle;
            }
            else{
                angle = 180 - angle;
                angle /= 270;
                angle = IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE.position.get() + angle;
            }
            MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPositionVoid(angle);
        }
    }

    @Override
    public void end(boolean interrupted) {
        FtcDashboard.getInstance().getTelemetry().addData("angle rotate = ",angle);
    }
}
