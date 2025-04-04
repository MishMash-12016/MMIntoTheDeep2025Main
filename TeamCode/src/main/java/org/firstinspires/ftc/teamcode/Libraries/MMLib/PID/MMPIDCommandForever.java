package org.firstinspires.ftc.teamcode.Libraries.MMLib.PID;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.utils.SQPIDController;

import java.util.function.DoubleSupplier;

public class MMPIDCommandForever extends CommandBase {

    public final MMPIDSubsystem subsystem;
    private final SQPIDController pidController;
    double setPoint = 0;


    public MMPIDCommandForever(MMPIDSubsystem subsystem) {
        this.subsystem = subsystem;
        this.pidController = subsystem.getPidController();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        setPoint = subsystem.getCurrentValue();
    }

    @Override
    public void execute() {
        if (!subsystem.doPid) return;

        pidController.setSetpoint(setPoint);
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
