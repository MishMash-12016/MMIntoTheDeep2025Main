package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.ClawSetState;
import org.firstinspires.ftc.teamcode.Commands.ScoringArmSetState;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;

public class ElevatorBackTo_0 extends SequentialCommandGroup {

    public ElevatorBackTo_0(){
        super(
                new ClawSetState(Claw.State.OPEN),
                new WaitCommand(300),
                new ScoringArmSetState(ScoringArm.Position.IN),
                new WaitCommand(300),
                new MMPIDCommand(MMRobot.getInstance().mmSystems.elevator,10),
                new WaitCommand(100),
                new MMPIDCommand(MMRobot.getInstance().mmSystems.elevator,1)
        );
        addRequirements(
                MMRobot.getInstance().mmSystems.elevator,
                MMRobot.getInstance().mmSystems.scoringArm,
                MMRobot.getInstance().mmSystems.claw
        );
    }

}
