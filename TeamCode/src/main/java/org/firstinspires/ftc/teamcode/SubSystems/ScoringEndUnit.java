package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class ScoringEndUnit extends SubsystemBase {
    CuttleServo clawscoringServo;
    CuttleServo scoringArmServo;

    public static double open = 1;
    public static double close = -1;
    public static double scoringHold = -1;
    public static double scoringSampleScore = 1;
    public static double scoringSpecimen=2;

    public ScoringEndUnit() {
        clawscoringServo = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.scoringClawServo);
        scoringArmServo = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.posscoringServo);
    }

    public Command openScoringClaw() {
        return new InstantCommand(() -> clawscoringServo.setPosition(open), this);

    }

    public Command closeScoringClaw() {
        return new InstantCommand(() -> clawscoringServo.setPosition(close), this);

    }

    public Command scoreSample() {
        return new InstantCommand(() -> scoringArmServo.setPosition(scoringSampleScore), this);
    }
    public Command scoreSpecimen(){
        return new InstantCommand(()-> scoringArmServo.setPosition(scoringSpecimen), this);
    }

    public Command scoringArmHold() {
        return new InstantCommand(() -> scoringArmServo.setPosition(scoringHold), this);
    }
}
