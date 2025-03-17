package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.Supplier;

@Config
public class ScoringClawEndUnit extends SubsystemBase {
    public static double scoringClawOpenPos = 0.95;
    public static double scoringClawClosePos = 0.65;
    public static double scoringClawEntirelyOpenPos = 1;
    CuttleServo clawScoringServo;
    public enum ScoringClawState {
        OPEN(()-> scoringClawOpenPos),
        CLOSE(()-> scoringClawClosePos),
        COMPLETELYOPEN(()-> scoringClawEntirelyOpenPos);//0.16
        public Supplier<Double> position;

        ScoringClawState(Supplier<Double> position) {
            this.position = position;
        }}


    public ScoringClawEndUnit() {
        clawScoringServo = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.SCORING_CLAW_SERVO);
        //clawScoringServo = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "Outake claw");
        clawScoringServo.setPosition(ScoringClawState.CLOSE.position.get());
    }

    public Command openScoringClaw() {
        return new InstantCommand(() -> {
            clawScoringServo.setPosition(ScoringClawState.OPEN.position.get());}, this);
    }
    public Command closeScoringClaw() {
        return new InstantCommand(() -> {
            clawScoringServo.setPosition(ScoringClawState.CLOSE.position.get());}, this);
    }

    public Command setPosition(double newPos){
        return new InstantCommand(()-> {
            clawScoringServo.setPosition(newPos);} ,
                this);
    }
    public Command setPosition(ScoringClawState scoringClawState){
        return new InstantCommand(()-> {
            clawScoringServo.setPosition(scoringClawState.position.get());} ,
                this);
    }
}
