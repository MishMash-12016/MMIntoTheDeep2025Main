package org.firstinspires.ftc.teamcode.CommandGroup;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;

public class ScoreSpecimenCommand {
    public static Command ScoreSpecimen() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.AFTER_SCORE_POSE)
        );
    }
}
