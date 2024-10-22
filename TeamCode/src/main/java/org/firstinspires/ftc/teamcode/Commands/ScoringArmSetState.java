package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;

public class ScoringArmSetState extends InstantCommand {


    public ScoringArmSetState(ScoringArm.Position position) {
        super(() -> MMRobot.getInstance().mmSystems.scoringArm.setPosition(position.scoringArmPosition));


    }
}
