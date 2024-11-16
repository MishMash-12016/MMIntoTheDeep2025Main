package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class ScoringEndUnit extends SubsystemBase {
    CuttleServo clawscoringServo;
    CuttleServo posscoringServo;

    public static double open = 1;
    public static double close = -1;
    public static double scoring_hold = -1;
    public static double scoring_score = 1;

    public ScoringEndUnit() {
        clawscoringServo = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.scoringClawServo);
        posscoringServo = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.posscoringServo);
    }

    public Command openScoringClaw() {
        return new InstantCommand(() -> clawscoringServo.setPosition(open), this);

    }
    public Command closeScoringClaw() {
        return new InstantCommand(() -> clawscoringServo.setPosition(close), this);

    }
    public Command scoreScoringServo(){
        return new InstantCommand(()-> posscoringServo.setPosition(scoring_score),this);
    }
    public Command holdScoringServo(){
        return new InstantCommand(()-> posscoringServo.setPosition(scoring_hold),this);
    }
}
