package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class ScoringClawEndUnit extends SubsystemBase {
     public CuttleServo clawScoringServo;
    public enum ScoringClawState {
        OPEN(0.4), CLOSE(0);
        public double position;
        ScoringClawState(double position){
            this.position = position;
        }}


    public ScoringClawEndUnit() {
        clawScoringServo = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.SCORING_CLAW_SERVO);
    }

    public Command openScoringClaw() {
        return new InstantCommand(() -> {
            clawScoringServo.enablePWM(true);
            clawScoringServo.setPosition(ScoringClawState.OPEN.position);}, this);
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
                clawScoringServo.setPosition(ScoringClawState.CLOSE.position);}, this);
    }
    public  Command disablePWM(){
        return new InstantCommand(()->
                clawScoringServo.enablePWM(false));
    }
}
