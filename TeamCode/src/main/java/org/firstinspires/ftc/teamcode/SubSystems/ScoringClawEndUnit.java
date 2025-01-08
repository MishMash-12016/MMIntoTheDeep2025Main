package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class ScoringClawEndUnit extends SubsystemBase {
     public CuttleServo clawScoringServo;


    public static double open = 0.9;
    public static double close = 0;
    public static double barelyopen = 0.3;


    public ScoringClawEndUnit() {
        clawScoringServo = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.SCORING_CLAW_SERVO);
    }

    public Command openScoringClaw() {
        return new InstantCommand(() -> {
            clawScoringServo.enablePWM(true);
            clawScoringServo.setPosition(open);}, this);
    }


    public Command setPosition(double newPos){
        return new InstantCommand(()-> {
            clawScoringServo.enablePWM(true);
            clawScoringServo.setPosition(newPos);} ,
                this);
    }

    public Command closeScoringClaw() {
        return new InstantCommand(() -> {
                clawScoringServo.enablePWM(true);
                clawScoringServo.setPosition(close);}, this);
    }
    public  Command disablePWM(){
        return new InstantCommand(()->
                clawScoringServo.enablePWM(false));
    }
}
