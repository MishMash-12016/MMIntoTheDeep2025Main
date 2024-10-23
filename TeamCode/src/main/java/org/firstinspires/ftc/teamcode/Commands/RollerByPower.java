package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.MMRobot;

public class RollerByPower extends InstantCommand {
    double power;

    public RollerByPower(double power) {
        super(()->MMRobot.getInstance().mmSystems.rollerIntake.setPower(power));
    }
}
