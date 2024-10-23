package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class LinearIntake extends SubsystemBase {

    CuttleServo rightServo;
    CuttleServo leftServo;

    public final static double OPEN = 0.34;
    public final static double SEMI_CLOSE = 0.13;

    public final static double CLOSE = 0;


    public LinearIntake() {
        rightServo = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.RIGHT_LINEAR_INTAKE);
        leftServo = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.LEFT_LINEAR_INTAKE);
    }
    public void setPosition(double position) {
        rightServo.setPosition(position);
        leftServo.setPosition (1-position);
    }

    public double getPosition() {
        return rightServo.getPosition();
    }

    public void updateTelemetry(){
        FtcDashboard.getInstance().getTelemetry().addData("rihgt",rightServo.getPosition());
        FtcDashboard.getInstance().getTelemetry().addData("left",leftServo.getPosition());
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public void periodic() {
        super.periodic();
        updateTelemetry();
    }
}
