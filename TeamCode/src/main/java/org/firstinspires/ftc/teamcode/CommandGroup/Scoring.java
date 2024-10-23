package org.firstinspires.ftc.teamcode.CommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.ClawSetState;
import org.firstinspires.ftc.teamcode.Commands.ElevatorGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.ScoringArmSetState;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;

public class Scoring extends SequentialCommandGroup {
    public Scoring(double high){
        addCommands(
                new ClawSetState(Claw.State.CLOSE),
                new WaitCommand(600),
                new ElevatorGetToPosition(high),
                new ScoringArmSetState(ScoringArm.Position.SCORING)
        );
        addRequirements(
                MMRobot.getInstance().mmSystems.elevator,
                MMRobot.getInstance().mmSystems.scoringArm,
                MMRobot.getInstance().mmSystems.claw
        );
    }
}
