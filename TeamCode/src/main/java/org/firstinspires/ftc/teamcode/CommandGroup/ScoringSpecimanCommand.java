package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;

public class ScoringSpecimanCommand {
    public static Command PrepareSpecimanScore(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoreSpecimen)
        );
    }
    public static Command SpecimanScore(){
        return new SequentialCommandGroup(
            MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoreSpecimen),
            MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()

                );


    }
}
