package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.IntakeArmSetState;
import org.firstinspires.ftc.teamcode.Commands.RollerByPower;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;

public class OutTakeRoller extends SequentialCommandGroup {

    public OutTakeRoller(){
        super(
                new IntakeArmSetState(IntakeArm.Position.OUT),
                new RollerByPower(-1),
                new WaitCommand(2000),
                new RollerByPower(0),
                new IntakeArmSetState(IntakeArm.Position.IN)
        );
    }

}
