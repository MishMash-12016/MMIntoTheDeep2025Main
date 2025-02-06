package org.firstinspires.ftc.teamcode.Libraries.MMLib.PID;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import java.util.function.DoubleSupplier;

public class MMPIDCommandForever extends CommandBase {

    private final MMPIDSubsystem subsystem;
    private final DoubleSupplier setPoint;
    private final PIDController pidController;

    public MMPIDCommandForever(MMPIDSubsystem subsystem, DoubleSupplier setPoint) {
        this.subsystem = subsystem;
        this.setPoint = setPoint;
        this.pidController = subsystem.getPidController();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        pidController.setSetPoint(setPoint.getAsDouble());
        subsystem.setPower(pidController.calculate(subsystem.getCurrentValue()) + subsystem.getFeedForwardPower());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }
}
