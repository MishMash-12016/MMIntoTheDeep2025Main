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
public class strafeToSample extends CommandBase {
    VoltageSensor voltageSensor;
    boolean finished;
    public strafeToSample(HardwareMap hardwareMap) {
        finished = false;
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
        double distanceX = MMRobot.getInstance().mmSystems.vision.getStrafeOffset() ;
        if (distanceX != 0){
            MMRobot.getInstance().mmSystems.driveTrain.actionBuilder(MMRobot.getInstance().mmSystems.currentPose).
                    lineToX(distanceX).
                    build();
        }
        finished = true;
    }


    @Override
    public void end(boolean interrupted) {
        MMRobot.getInstance().mmSystems.vision.stopTracking();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
