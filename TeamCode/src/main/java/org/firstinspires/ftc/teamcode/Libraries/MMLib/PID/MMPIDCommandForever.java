package org.firstinspires.ftc.teamcode.Libraries.MMLib.PID;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.utils.SQPIDController;

import java.util.function.DoubleSupplier;

@Config
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
        pidController.reset();
        pidController.setSetpoint(MMRobot.getInstance().mmSystems.elevator.targetPose);
    }

    @Override
    public void execute() {

//        if (pidController.atSetpoint()){
//            subsystem.setPower(0.0);
//        }else {
//            subsystem.setPower(pidController.calculate(subsystem.getCurrentValue()) + subsystem.getFeedForwardPower());
//        }

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
