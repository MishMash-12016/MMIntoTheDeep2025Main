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
    public static  double Kp_short = 0.1;
    public static  double Ki_short = 0.042;
    public static  double Kd_short = 0.0055;
    public static  double Ks_short = 2;

    public static  double Kp_long = 0.1;
    public static  double Ki_long = 0.042;
    public static  double Kd_long = 0.0055;
    public static  double Ks_long = 1.32;

    public static  double tolerance = 0.5;
    public static double timeAligned = 150;
    public static double setPoint = 0;

    SQPIDController pidControllerLong;
    SQPIDController pidControllerShort;
    ElapsedTime timer;
    VoltageSensor voltageSensor;

    public static double min = -16;
    public static double max = 16;
    boolean shortPID = false;
    boolean longPID = false;

    public alignToSample(HardwareMap hardwareMap) {
        addRequirements(
                MMRobot.getInstance().mmSystems.driveTrain);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }


    @Override
    public void initialize() {
        pidControllerShort = new SQPIDController(Kp_short, Ki_short, Kd_short, Ks_short,0,0); //better for small distances
        pidControllerShort.setSetpoint(setPoint);
        pidControllerShort.setTolerance(tolerance);

        pidControllerLong = new SQPIDController(Kp_long, Ki_long, Kd_long, Ks_long,0,0); //better for small distances
        pidControllerLong.setSetpoint(setPoint);
        pidControllerLong.setTolerance(tolerance);

        timer = new ElapsedTime();
        timer.reset();
        MMRobot.getInstance().mmSystems.vision.startTracking();

    }

    @Override
    public void execute() {
        double distnaceX = -MMRobot.getInstance().mmSystems.vision.getTx(0);

        // PID updating based on limelight distance from target in the x axis, two PID for short distance and long:
        if (distnaceX >= min && distnaceX <=max && !longPID){
            MMRobot.getInstance().mmSystems.driveTrain.drive(0, 0,
                    pidControllerShort.calculate(distnaceX)/voltageSensor.getVoltage());
            if (!shortPID){
                shortPID = true;
            }
        }
        else {
            if (!shortPID) {
                MMRobot.getInstance().mmSystems.driveTrain.drive(0, 0,
                        pidControllerLong.calculate(distnaceX) / voltageSensor.getVoltage());
                longPID = true;
            }
        }

        //if the pid controller isn't at the set point the timer will reset:
        if (!pidControllerLong.atSetpoint() && !pidControllerShort.atSetpoint()){
            timer.reset();
        }
        FtcDashboard.getInstance().getTelemetry().addData("timer", timer.milliseconds());
    }


    @Override
    public void end(boolean interrupted) {
        MMRobot.getInstance().mmSystems.driveTrain.drive(0,0,0); //Stops the robot when the pid finished:
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= timeAligned; //meaning it have been in the set pint for a time so it can be stopped
    }
}
