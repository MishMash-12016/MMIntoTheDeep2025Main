package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.MMRobot;

public class SetLinearPosition extends InstantCommand {

    public SetLinearPosition(double position){
        super(()-> MMRobot.getInstance().mmSystems.linearIntake.setPosition(position));
    }


}
