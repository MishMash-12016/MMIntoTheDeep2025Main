
package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.function.Supplier;

@Config
public class ScoringEndUnitElbow extends SubsystemBase {
    public static double restPos = 0.73;
    public static double transferSpecimenPos = 0.4;
    public static double transferSamplePos = 0.45;
    public static double scoreSamplePos = 0.61;
    public static double initPos = 0.4 ;
    public static double prepareSampleTransfer = 0.7;
    public static double scoreSpecimenPos = 0.98;
    public static double intakeFromBackPos = 0.98;


    private final static MMRobot robotInstance = MMRobot.getInstance();
    public enum ScoringElbowState {
        REST_POSE(()-> restPos),
        TRANSFER_SPECIMEN_POSE(()-> transferSpecimenPos),
        PREPARE_SAMPLE_TRANSFER(()-> prepareSampleTransfer),
        TRANSFER_SAMPLE_POSE(()-> transferSamplePos),
        SCORE_SAMPLE_POSE(()-> scoreSamplePos),
        INIT_POSE(()-> initPos),
        SCORE_SPECIMEN_POSE(()-> scoreSpecimenPos),
        INTAKE_FROM_BACK_POSE(()-> intakeFromBackPos);


        public Supplier<Double> position;

        ScoringElbowState(Supplier<Double> position) {
            this.position = position;
        } }
    Servo servo;

    public ScoringEndUnitElbow(){
        servo = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "scoring rot");//0
        //servo = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.SCORING_ROTATOR_SERVO);
        //servo = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "Outake angle");
        servo.setPosition(ScoringElbowState.INIT_POSE.position.get());
    }

    public Command setPosition(double newPos){
        return new InstantCommand(()-> {
            servo.setPosition(newPos);} ,
                this);
    }
    public Command setPosition(ScoringElbowState state){
        return new InstantCommand(()-> {
            servo.setPosition(state.position.get());} ,
                this);
    }
}

