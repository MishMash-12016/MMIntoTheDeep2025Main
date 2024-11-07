package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class ScoringEndUnit extends SubsystemBase {
    CuttleServo scoringClawServo;

    public static double open = 1;
    public static double close = -1;

    public ScoringEndUnit(){
        scoringClawServo= new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.scoringClawServo);
    }
    public Command openScoringClaw() {
        return new RunCommand(()->scoringClawServo.setPosition(open),this);

    }
    public Command closeScoringClaw() {
        return new RunCommand(()-> scoringClawServo.setPosition(close));

    }
}
