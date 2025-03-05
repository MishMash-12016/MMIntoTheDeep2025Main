package org.firstinspires.ftc.teamcode.CommandGroup;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.MMRobot;

public class ScoreSpecimenCommand {
    public static Command ScoreSpecimen() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
        );
    }
}
