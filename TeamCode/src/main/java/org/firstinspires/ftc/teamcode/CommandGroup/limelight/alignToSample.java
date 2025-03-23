package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.SQPIDController;

@Config
public class alignToSample extends CommandBase {
    VoltageSensor voltageSensor;

    public alignToSample(HardwareMap hardwareMap) {
        addRequirements(
                MMRobot.getInstance().mmSystems.driveTrain);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }


    @Override
    public void initialize() {
        MMRobot.getInstance().mmSystems.vision.startTracking();
    }

    @Override
    public void execute() {
        double distanceX = MMRobot.getInstance().mmSystems.vision.getTx(0);
        if (distanceX != 0){
            MMRobot.getInstance().mmSystems.driveTrain.actionBuilder(MMRobot.getInstance().mmSystems.currentPose).
                    turnTo(distanceX).
                    build();
        }
    }

    @Override
    public void end(boolean interrupted) {
        MMRobot.getInstance().mmSystems.vision.stopTracking();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}