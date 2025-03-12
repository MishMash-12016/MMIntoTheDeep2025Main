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

    // PID:
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

    public alignToSample(HardwareMap hardwareMap) {
        addRequirements(
                MMRobot.getInstance().mmSystems.driveTrain);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }


    @Override
    public void initialize() {
        pidController = new SQPIDController(Kp,Ki,Kd,Ks,0,0); //better for small distances
        pidController.setSetpoint(setPoint);
        pidController.setTolerance(tolerance);
        timer = new ElapsedTime();
        timer.reset();
        MMRobot.getInstance().mmSystems.vision.startTracking();

    }

    @Override
    public void execute() {
        // PID updating based on limelight distance from target in the x axis:
        MMRobot.getInstance().mmSystems.driveTrain.drive(0, 0,
                pidController.calculate(MMRobot.getInstance().mmSystems.vision.getTx(0))/voltageSensor.getVoltage());

        //if the pid controller isnt at the set point the timer will reset:
        if (!pidController.atSetpoint()){
            timer.reset();
        }
        FtcDashboard.getInstance().getTelemetry().addData("timer", timer.milliseconds());
    }


    @Override
    public void end(boolean interrupted) {
        MMRobot.getInstance().mmSystems.driveTrain.drive(0,0,0); //Stops the robot when the pid finished:
        MMRobot.getInstance().mmSystems.vision.stopTracking();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= timeAligned; //meaning it have been in the set pint for a time so it can be stopped
    }
}
