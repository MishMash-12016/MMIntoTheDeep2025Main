package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Commands.IntakeArmSetState;
import org.firstinspires.ftc.teamcode.Commands.RollerByPower;
import org.firstinspires.ftc.teamcode.Commands.LinearIntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.SetLinearPosition;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;

public class Intake extends ParallelDeadlineGroup {

    public Intake(Trigger trigger) {
        super(
                new LinearIntakeCommand(trigger),
                new IntakeArmSetState(IntakeArm.Position.OUT),
                new RollerByPower(1)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        CommandScheduler.getInstance().schedule(
                new IntakeArmSetState(IntakeArm.Position.MID),
                new WaitCommand(400),
                new IntakeArmSetState(IntakeArm.Position.IN),
                new RollerByPower(0.3),
                new SetLinearPosition(0.18),
                new WaitCommand(200),
                new SetLinearPosition(0),
                new RollerByPower(0)
        );


    }

}
