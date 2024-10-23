package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.ClawSetState;
import org.firstinspires.ftc.teamcode.Commands.IntakeArmSetState;
import org.firstinspires.ftc.teamcode.Commands.LinearIntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.RollerByPower;
import org.firstinspires.ftc.teamcode.Commands.SetLinearPosition;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;

public class AutoIntake extends SequentialCommandGroup {
    public AutoIntake(){
        super(
                new IntakeArmSetState(IntakeArm.Position.OUT),
                new WaitCommand(700),
                new RollerByPower(1),
                new WaitCommand(600),
                new SetLinearPosition(LinearIntake.OPEN),
                new WaitCommand(3000),
                new RollerByPower(0),
                new IntakeArmSetState(IntakeArm.Position.MID),
                new WaitCommand(600),
                new SetLinearPosition(LinearIntake.SEMI_CLOSE),
                new WaitCommand(600),
                new IntakeArmSetState(IntakeArm.Position.IN),
                new WaitCommand(600),
                new SetLinearPosition(LinearIntake.CLOSE),
                new WaitCommand(3000),
                new ClawSetState(Claw.State.CLOSE),
                new RollerByPower(-0.3),
                new WaitCommand(500),
                new RollerByPower(0)
        );
    }
}
