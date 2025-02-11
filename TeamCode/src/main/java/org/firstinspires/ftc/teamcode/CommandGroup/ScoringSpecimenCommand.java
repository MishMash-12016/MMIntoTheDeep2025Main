package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm.ScoringArmState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator.ScoringRotatorState;

public class ScoringSpecimenCommand {
    public static Command SpecimenScore(){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.TRANSFER_SAMPLE_POSE),
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw()
                ),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.MID_POSE_SPECIMEN),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.SCORE_SPECIMEN),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.SCORE_SPECIMEN_POSE)
        );
    }

}
