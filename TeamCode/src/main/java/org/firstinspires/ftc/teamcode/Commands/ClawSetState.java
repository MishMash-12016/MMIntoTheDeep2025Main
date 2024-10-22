package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;

public class ClawSetState extends InstantCommand {

    public ClawSetState(Claw.State state){
        super(()-> MMRobot.getInstance().mmSystems.claw.setPosition(state.position));
    }

}
