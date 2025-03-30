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
    public static double scoringArmInitPose = 0.55;
    public static double scoringArmSampleTransferPose = 0.3;
    public static double scoringArmPrepareSampleTransferPose = 0.65;
    public static double scoringArmScorePose = 0.9;
    public static double scoringArmIntakeFromFrontPose = 0.55;
    public static double scoringArmSpecimenSideScore = 0.725;
    public static double scoringArmScoreFromFrontSpecimenPose = 0.6;

    public double estimatedPose = scoringArmInitPose;

    public enum ScoringArmState {
        INIT_POSE(() -> scoringArmInitPose),
        SAMPLE_TRANSFER_POSE(() -> scoringArmSampleTransferPose),
        SCORE_FROM_FRONT(()-> scoringArmScoreFromFrontSpecimenPose),
        SCORING_ARM_SCORE_POSE(()-> scoringArmScorePose),
        ARM_PREPARE_SAMPLE_TRANSFER_POSE(()-> scoringArmPrepareSampleTransferPose),
        INTAKE_FROM_FRONT_POSE(()-> scoringArmIntakeFromFrontPose);

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
        servoLeft.setPosition(ScoringArmState.INIT_POSE.position.get()+0.015);
        servoRight.setPosition(1 - ScoringArmState.INIT_POSE.position.get());
    }

    //Tell arm to get to position
    public Command setPosition(double newPos) {
        estimatedPose = newPos;
        return new InstantCommand(() -> {
            servoLeft.setPosition(newPos+0.015);
            servoRight.setDirection(Servo.Direction.REVERSE);
        },
                this);
    }


    public Command setPosition(ScoringArmState state) {
        return new InstantCommand(() -> setPosition(state.position.get()));
    }
}