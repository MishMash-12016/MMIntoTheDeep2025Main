package org.firstinspires.ftc.teamcode.CommandGroup;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MMRobot;


@Config
public class ScoringArm_SpeedControll extends CommandBase {
    ElapsedTime currentTimeMS;
    double neededTimeMS;
    double finalPose;
    double diffrance;
    double startPose;
    public ScoringArm_SpeedControll(double neededTimeMS, double pose) {
        this.finalPose = pose;
        this.neededTimeMS = neededTimeMS;
        currentTimeMS = new ElapsedTime();

        addRequirements(
                MMRobot.getInstance().mmSystems.scoringArm);
    }


    @Override
    public void initialize() {
        currentTimeMS.reset();
        startPose = MMRobot.getInstance().mmSystems.scoringArm.getPosition();
        diffrance = finalPose - MMRobot.getInstance().mmSystems.scoringArm.getPosition();

    }

    @Override
    public void execute() {
        MMRobot.getInstance().mmSystems.scoringArm.setPositionVoid(startPose + (currentTimeMS.milliseconds() / neededTimeMS) * diffrance);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return currentTimeMS.milliseconds() >= neededTimeMS;
    }
}