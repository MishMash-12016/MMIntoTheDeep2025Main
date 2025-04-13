package org.firstinspires.ftc.teamcode.CommandGroup;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MMRobot;


@Config
public class IntakeArm_SpeedControl extends CommandBase {
    ElapsedTime currentTimeMS;
    double neededTimeMS;
    double finalPose;
    double diffrance;
    double startPose;
    public IntakeArm_SpeedControl(double neededTimeMS, double pose) {
        this.finalPose = pose;
        this.neededTimeMS = neededTimeMS;
        currentTimeMS = new ElapsedTime();

        addRequirements(
                MMRobot.getInstance().mmSystems.intakeArm);
    }


    @Override
    public void initialize() {
        currentTimeMS.reset();
        startPose = MMRobot.getInstance().mmSystems.intakeArm.getPosition();
        diffrance = finalPose - MMRobot.getInstance().mmSystems.intakeArm.getPosition();

    }

    @Override
    public void execute() {
        MMRobot.getInstance().mmSystems.intakeArm.setPositionVoid(startPose + (currentTimeMS.milliseconds() / neededTimeMS) * diffrance);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return currentTimeMS.milliseconds() >= neededTimeMS;
    }
}