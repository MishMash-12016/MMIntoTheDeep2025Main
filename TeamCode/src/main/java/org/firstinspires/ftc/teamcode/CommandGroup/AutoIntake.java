package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.LinearIntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.SetLinearPosition;

public class AutoIntake extends SequentialCommandGroup {
    public AutoIntake(){
        super(
                new SetLinearPosition(0.8),
                new IntakeArmDownAndStartRoller().withTimeout(3000),
                new SetLinearPosition(0)

        );
    }

}
