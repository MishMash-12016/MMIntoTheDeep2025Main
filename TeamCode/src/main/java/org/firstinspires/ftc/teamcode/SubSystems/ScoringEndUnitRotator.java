
package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Config
public class ScoringEndUnitRotator extends SubsystemBase {
    public static double restPos = 0.45;
    public static double transferPos = 0.13;
    public static double transferSamplePos = 0.18;
    public static double scoreSamplePos = 0.33;
    public static double initPos = 0.23;
    public static double midposespecimenPos = 0.23;
    public static double scoreSpecimenPos = 0.72;

    private final static MMRobot robotInstance = MMRobot.getInstance();
    public enum ScoringRotatorState {
        REST_POSE(()-> restPos),
        TRANSFER_POSE(()-> transferPos),
        TRANSFER_SAMPLE_POSE(()-> transferSamplePos),
        SCORE_SAMPLE_POSE(()-> scoreSamplePos),
        INIT_POSE(()-> initPos),
        MID_POSE_SPECIMEN(()-> midposespecimenPos),
        SCORE_SPECIMEN_POSE(()-> scoreSpecimenPos);


        public Supplier<Double> position;

        ScoringRotatorState(Supplier<Double> position) {
            this.position = position;
        } }
    Servo servo;

    public ScoringEndUnitRotator(){
        servo = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "scoring rot");//0
        //servo = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.SCORING_ROTATOR_SERVO);
        //servo = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "Outake angle");
        servo.setPosition(ScoringRotatorState.INIT_POSE.position.get());
    }

    public Command setPosition(double newPos){
        return new InstantCommand(()-> {
            servo.setPosition(newPos);} ,
                this);
    }
    public Command setPosition(ScoringRotatorState state){
        return new InstantCommand(()-> {
            servo.setPosition(state.position.get());} ,
                this);
    }
}

