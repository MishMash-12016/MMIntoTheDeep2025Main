package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Libraries.exterpolation.ExterpolationMap;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;

public class RotateIntakeServoByRobotAngle extends CommandBase {
    double startRobotAngle,
            endRobotAngle,
            startIntakeAngle,
            endIntakeAngle;

    ExterpolationMap exterpolationMap;
    MMSystems mmSystems;


    public RotateIntakeServoByRobotAngle(double startRobotAngle, double endRobotAngle, double startIntakeAngle, double endIntakeAngle) {
        this.startRobotAngle = startRobotAngle;
        this.endRobotAngle = endRobotAngle;
        this.startIntakeAngle = startIntakeAngle;
        this.endIntakeAngle = endIntakeAngle;

        exterpolationMap = new ExterpolationMap()
                .put(startRobotAngle, startIntakeAngle)
                .put(endRobotAngle, endIntakeAngle);

        addRequirements(MMRobot.getInstance().mmSystems.intakeEndUnitRotator);
    }

    @Override
    public void initialize() {
        mmSystems = MMRobot.getInstance().mmSystems;
    }

    @Override
    public void execute() {
        mmSystems.intakeEndUnitRotator.setPositionVoid(
                exterpolationMap.exterpolate(mmSystems.driveTrain.pinpoint.getHeading())
        );
    }
}
