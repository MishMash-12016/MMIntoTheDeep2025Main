package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.MMRobot;

public class ElevatorClimb extends CommandBase {

    public ElevatorClimb(){}

    @Override
    public void initialize() {
        super.initialize();
        MMRobot.getInstance().mmSystems.elevator.setPower((double) -1);
    }

    @Override
    public boolean isFinished() {
        return MMRobot.getInstance().mmSystems.elevator.getHeight() == 0;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        MMRobot.getInstance().mmSystems.elevator.setPower((double) 0);
    }

}
