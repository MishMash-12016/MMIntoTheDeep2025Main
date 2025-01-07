package org.firstinspires.ftc.teamcode.CommandGroup;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;

public class ScoreSampleCommand {
    public static Command ScoreSampleToHigh(){
        return new SequentialCommandGroup(
                new MMPIDCommand(MMRobot.getInstance().mmSystems.elevator, 10), //the height of the high basket
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoreSpecimen)
        );
    }
    public static Command ScoreSampleToLow(){
        return new SequentialCommandGroup(
                new MMPIDCommand(MMRobot.getInstance().mmSystems.elevator, 5), //the height of the low basket
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoreSpecimen)
        );
    }
    //add somewhere that the scoring claw will open
    public static Command backToIntake(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.transferHold),
                new MMPIDCommand(MMRobot.getInstance().mmSystems.elevator,20),
                new WaitCommand(200),
                new MMPIDCommand(MMRobot.getInstance().mmSystems.elevator,0)

        );

    }
}
