package org.firstinspires.ftc.teamcode.CommandGroup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Libraries.exterpolation.ExterpolationMap;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;

public class RotateIntakeServoByRobotAngle extends CommandBase {
    ExterpolationMap exterpolationMap;
    MMSystems mmSystems;


    public RotateIntakeServoByRobotAngle(double startRobotAngle, double endRobotAngle, double startIntakeAngle, double endIntakeAngle) {

        this.exterpolationMap = new ExterpolationMap()
                .put(startRobotAngle, startIntakeAngle)
                .put(180, endIntakeAngle)
                .put(endRobotAngle, startRobotAngle);

        addRequirements(MMRobot.getInstance().mmSystems.intakeEndUnitRotator);
    }

    @Override
    public void initialize() {
        mmSystems = MMRobot.getInstance().mmSystems;
    }

    @Override
    public void execute() {
        double robotAngel = Math.toDegrees(mmSystems.driveTrain.pose.heading.toDouble());
        if(robotAngel < 0){
            robotAngel += 360;
        }

        double rotatorAngle = exterpolationMap.exterpolate(robotAngel);
        mmSystems.intakeEndUnitRotator.setPositionVoid(rotatorAngle);

        FtcDashboard.getInstance().getTelemetry().addData("robotAngle", Math.toDegrees(mmSystems.driveTrain.pose.heading.toDouble()));
        FtcDashboard.getInstance().getTelemetry().addData("rotatorAngle", rotatorAngle);
    }

}
