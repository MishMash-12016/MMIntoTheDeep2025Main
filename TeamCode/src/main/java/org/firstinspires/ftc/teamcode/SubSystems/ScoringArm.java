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
    public static double scoringArmInitPose = 0.4- 0.28;
    public static double scoringArmMidPose = 0.49;
    public static double scoringArmRestPose = 0.46- 0.28+0.12-0.12;

    public static double scoringArmSampleTransferPose = 0.3;
    public static double scoringArmPrepareSampleTransferPose = 0.65;
    public static double scoringArmSpecimenScorePose = 0.63-0.12;
    public static double scoringArmSampleScorePose = 0.48;
    public static double scoringArmScorePose = 0.9;
    public static double scoringArmIntakeFromFrontPose = 0.55;
    public static double scoringArmAfterScoreSpecimenPose = 0.55;
    public static double scoringArmSpecimenSideScore = 0.725;
    public static double scoringArmScoreFromFrontSpecimenPose = 0.6;
    public static double scoringArmPrepareScoreFromFrontSpecimenPose = 0.75;

    public static double scoringArmAfterScoreFromFrontSpecimenPose = 0.17;

    public double estimatedPose = scoringArmInitPose;

    public enum ScoringArmState {
        MID_POSE(() -> scoringArmMidPose),
        REST_POSE(() -> scoringArmRestPose),
        SAMPLE_TRANSFER_POSE(() -> scoringArmSampleTransferPose),
        INIT_POSE(() -> scoringArmInitPose),
        SCORE_SPECIMEN(() -> scoringArmSpecimenScorePose),
        SCORE_SAMPLE(() -> scoringArmSampleScorePose),
        SCORE_FROM_FRONT(()-> scoringArmScoreFromFrontSpecimenPose),
        SCORING_ARM_SCORE_POSE(()-> scoringArmScorePose),
        ARM_PREPARE_SAMPLE_TRANSFER_POSE(()-> scoringArmPrepareSampleTransferPose),
        AFTER_SCORE_POSE(()-> scoringArmAfterScoreSpecimenPose),
        INTAKE_FROM_FRONT_POSE(()-> scoringArmIntakeFromFrontPose),
        SCORING_SPECIMEN_SIDE_POSE(() -> scoringArmSpecimenSideScore),
        AFTER_SCORING_FRONT_SPECIMEN_POSE(() -> scoringArmAfterScoreFromFrontSpecimenPose),
        PREPARE_SCORE_FROM_FRONT_SPECIMEN_POSE(()-> scoringArmPrepareScoreFromFrontSpecimenPose);

        public final Supplier<Double> position;

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
        estimatedPose = newPos+0.01;
        return new InstantCommand(() -> {
            servoLeft.setPosition(newPos+0.015);
            servoRight.setDirection(Servo.Direction.REVERSE);
        },
                this);
    }


    public Command setPosition(ScoringArmState state) {
        estimatedPose = state.position.get()+0.01;
        return new InstantCommand(() -> {
            servoLeft.setPosition(state.position.get()+0.01);
            servoRight.setPosition(1 - state.position.get());
        },
                this);
    }
}