package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.FTCTimer;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

import java.util.List;

public class strafeToSample extends CommandBase {
    PIDController pidController;
    FTCTimer timer;
    public strafeToSample() {
        addRequirements(
                MMRobot.getInstance().mmSystems.driveTrain);
    }


    @Override
    public void initialize() {
        pidController = new PIDController(0.0124, 0,0.0013);
        pidController.setSetPoint(0);
        pidController.setTolerance(7);
        timer = new FTCTimer();
        timer.start();
    }

    @Override
    public void execute() {
        MMRobot.getInstance().mmSystems.driveTrain.drive(pidController.calculate(MMRobot.getInstance().mmSystems.vision.getStrafeOffset()), 0, 0);
        if (!pidController.atSetPoint()){
            timer.start();
        }
        MMRobot.getInstance().mmSystems.telemetry.addData("tim,er", timer.getTime());
    }

    @Override
    public void end(boolean interrupted) {
        MMRobot.getInstance().mmSystems.driveTrain.drive(0,0,0);
    }

    @Override
    public boolean isFinished() {
        timer.end();
        return timer.getTime() >= 300;

    }
}
