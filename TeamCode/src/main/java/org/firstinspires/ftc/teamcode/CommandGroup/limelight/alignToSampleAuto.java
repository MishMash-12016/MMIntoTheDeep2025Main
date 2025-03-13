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
    public static  double tolerance = 0.5;
    public static double timeAligned = 150;
    public static  double setPoint = 0;
    SQPIDController pidController;
    ElapsedTime timer;

    VoltageSensor voltageSensor;

    PinpointDrive drive;
    public alignToSampleAuto(HardwareMap hardwareMap , PinpointDrive drive) {
        this.drive = drive;
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }


    @Override
    public void initialize() {
        pidController = new SQPIDController(Kp, Ki,Kd,Ks,0,0);
        pidController.setSetpoint(setPoint);
        pidController.setTolerance(tolerance);
        timer = new ElapsedTime();
        timer.reset();
    }

    @Override
    public void execute() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),
                pidController.calculate(-MMRobot.getInstance().mmSystems.vision.getTx(0))
                        / voltageSensor.getVoltage()));
        if (!pidController.atSetpoint()){
            timer.reset();
        }
        FtcDashboard.getInstance().getTelemetry().addData("timer", timer.milliseconds());
    }


    @Override
    public void end(boolean interrupted) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= timeAligned;
    }
}
