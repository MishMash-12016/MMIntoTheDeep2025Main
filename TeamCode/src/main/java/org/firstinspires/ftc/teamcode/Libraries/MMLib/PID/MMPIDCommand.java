package org.firstinspires.ftc.teamcode.Libraries.MMLib.PID;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

public class MMPIDCommand extends CommandBase {

    private final MMPIDSubsystem subsystem;
    private final double setPoint;
    private final PIDController pidController;

    public MMPIDCommand(MMPIDSubsystem subsystem, double setPoint) {
        this.subsystem = subsystem;
        this.setPoint = setPoint;
        this.pidController = subsystem.getPidController();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        pidController.setSetPoint(setPoint);
    }

    @Override
    public void execute() {
        subsystem.setPower(pidController.calculate(subsystem.getCurrentValue()) + subsystem.getFeedForwardPower());
    }

    @Override
    public boolean isFinished() {
//        return false;
        return pidController.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }
}
