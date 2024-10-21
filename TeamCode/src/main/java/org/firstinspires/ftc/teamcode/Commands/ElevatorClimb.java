package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

public class ElevatorClimb extends CommandBase {

    Elevator elevator;

    public ElevatorClimb(Elevator elevator){
        this.elevator = elevator;
    }
    @Override
    public void initialize() {
        super.initialize();
        elevator.setPower((double) -1);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        elevator.setPower((double) 0);
    }
}
