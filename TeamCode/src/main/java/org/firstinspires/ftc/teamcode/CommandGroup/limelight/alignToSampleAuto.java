package org.firstinspires.ftc.teamcode.CommandGroup.limelight;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.SQPIDController;

@Config
public class alignToSampleAuto extends CommandBase {

    public static  double Kp = 0.1;
    public static  double Ki = 0.042;
    public static  double Kd = 0.0055;
    public static  double Ks = 1.32;
    public static  double tolerance = 1.0;
    public static double timeAligned = 150;
    SQPIDController pidController;
    ElapsedTime timer;

    PinpointDrive driveTrain;
    VoltageSensor voltageSensor;
    public alignToSampleAuto(HardwareMap hardwareMap , PinpointDrive driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(
                MMRobot.getInstance().mmSystems.driveTrain);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }


    @Override
    public void initialize() {
        pidController = new SQPIDController(Kp, Ki,Kd,Ks,0,0);
        pidController.setSetpoint(0);
        pidController.setTolerance(tolerance);
        timer = new ElapsedTime();
        timer.reset();
        MMRobot.getInstance().mmSystems.vision.startTracking();

    }

    @Override
    public void execute() {
        driveTrain.setDrivePowers(new PoseVelocity2d(
                new Vector2d(0,
                        pidController.calculate(-MMRobot.getInstance().mmSystems.vision.getTx(
                                0))/voltageSensor.getVoltage()),
                0));
        if (!pidController.atSetpoint()){
            timer.reset();
        }
        FtcDashboard.getInstance().getTelemetry().addData("timer", timer.milliseconds());
    }


    @Override
    public void end(boolean interrupted) {
        driveTrain.setDrivePowers(new PoseVelocity2d(
                new Vector2d(0,
                        0),
                0));
        MMRobot.getInstance().mmSystems.vision.stopTracking();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= timeAligned
                ;
    }
}
