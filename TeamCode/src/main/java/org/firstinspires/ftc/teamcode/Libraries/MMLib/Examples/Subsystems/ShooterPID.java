package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleEncoder;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.utils.Configuration;


@Config
public class ShooterPID extends MMPIDSubsystem {

    MMRobot mmRobot = MMRobot.getInstance();

    //hardware
    private final CuttleMotor motor;
    private final CuttleEncoder encoder;

    //control
    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double tolerance = 0;

    //constants
    private final double TICKS_PER_REV = 3895.9;

    public ShooterPID() {
        super(kP, kI, kD, tolerance);
        this.motor = mmRobot.mmSystems.controlHub.getMotor(Configuration.SHOOTER1);
        this.encoder = mmRobot.mmSystems.controlHub.getEncoder(Configuration.SHOOTER1, TICKS_PER_REV);
    }

    @Override
    public void setPower(Double power) {
        motor.setPower(power);
    }

    @Override
    public double getCurrentValue() {
        return encoder.getRPM();
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    @Override
    public void periodic() {
        telemetry();
    }

    private void telemetry() {
        mmRobot.mmSystems.telemetry.addData("Current Value", getCurrentValue());
        FtcDashboard.getInstance().getTelemetry().addData("Current Value", getCurrentValue());
        FtcDashboard.getInstance().getTelemetry().addData("Target", getPidController().getSetPoint());
    }

}
