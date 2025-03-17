package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.function.Supplier;

@Config
public class ScoringArm extends SubsystemBase {

    public static double scoringArmMidePose = 0.77 - 0.28+0.12-0.12;
    public static double scoringArmRestPose = 0.46- 0.28+0.12-0.12;
    public static double scoringArmSpecimenTransferPose = 0.62- 0.28+0.12-0.12;
    public static double scoringArmSampleTransferPose = 0.61- 0.28+0.12-0.12;
    public static double scoringArmInitPose = 0.4- 0.28 +0.12-0.12;
    public static double scoringArmSpecimenScorePose = 0.63-0.12;
    public static double scoringArmSampleScorePose = 0.77- 0.28+ 0.12-0.12;
    public static double scoringArmSamplePrepareScorePose = 0.33- 0.28+ 0.12-0.12;
    public static double scoringArmIntakeFromFrontPose = 0.19; //0.21
    public static double scoringArmAfterScoreSpecimenPose = 0.55;
    public static double scoringArmSpecimenSideScore = 0.725;
    public static double scoringArmScoreFromFrontSpecimenPose = 0.51-0.12;

    public static double scoringArmAfterScoreFromFrontSpecimenPose = 0.3-0.12;
    public static double scoringArmAfterScoreFromSideSpecimenPose = 0.7-0.12;
    public static double scoringArmMidToFront = 0.4-0.12;

    public enum ScoringArmState {
        MID_TO_FRONT(() -> scoringArmMidToFront),
        MID_POSE(() -> scoringArmMidePose),
        REST_POSE(() -> scoringArmRestPose),
        SPECIMEN_TRANSFER_POSE(() -> scoringArmSpecimenTransferPose),
        SAMPLE_TRANSFER_POSE(() -> scoringArmSampleTransferPose),
        INIT_POSE(() -> scoringArmInitPose),
        SCORE_SPECIMEN(() -> scoringArmSpecimenScorePose),
        SCORE_SAMPLE(() -> scoringArmSampleScorePose),
        SCORE_FROM_FRONT(()-> scoringArmScoreFromFrontSpecimenPose),
        PREPARE_SCORE_SAMPLE(() -> scoringArmSamplePrepareScorePose),
        AFTER_SCORE_POSE(()-> scoringArmAfterScoreSpecimenPose),
        INTAKE_FROM_FRONT_POSE(()-> scoringArmIntakeFromFrontPose),
        SCORING_SPECIMEN_SIDE_POSE(() -> scoringArmSpecimenSideScore),
        AFTER_SCORING_FRONT_SPECIMEN_POSE(() -> scoringArmAfterScoreFromFrontSpecimenPose),
        AFTER_SCORING_SIDE_SPECIMEN_POSE(() -> scoringArmAfterScoreFromSideSpecimenPose);

        public Supplier<Double> position;

        ScoringArmState(Supplier<Double> position) {
            this.position = position;
        }
    }

    Servo servoLeft;
    Servo servoRight;

    public ScoringArm() {
        servoLeft = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "L outake arm ");//1
        servoRight = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "R outake arm");//4
        servoLeft.setPosition(ScoringArmState.INIT_POSE.position.get()+0.01);
        servoRight.setPosition(1 - ScoringArmState.INIT_POSE.position.get());
    }

    //Tell arm to get to position
    public Command setPosition(double newPos) {
        return new InstantCommand(() -> {
            servoLeft.setPosition(newPos+0.01);
            servoRight.setPosition(1 - newPos);
        },
                this);
    }


    public Command setPosition(ScoringArmState state) {
        return new InstantCommand(() -> {
            servoLeft.setPosition(state.position.get()+0.01);
            servoRight.setPosition(1 - state.position.get());
        },
                this);
    }

    public double getPosition() {
        return servoRight.getPosition();
    }
}