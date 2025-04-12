package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.function.Supplier;

@Config
public class ScoringArm extends SubsystemBase {
    public static double scoringArmInitPose = 0.52;
    public static double scoringArmSampleTransferPose = 0.54;
    public static double scoringArmPrepareSampleTransferPose = 0.45;
    public static double scoringArmScorePose = 0.19;
    public static double scoringArmIntakeFromFrontPose = 0.545;
    public static double scoringArmScoreFromFrontSpecimenPose = 0.42;
    public static double afterScoreFromFrontSpecimenPose = 0.3;
    public static double scoringArmScoreSample = 0.24;
    public static double scoringArmPark = 0.39;
    public static double estimatedPose = scoringArmInitPose;

    public enum ScoringArmState {
        INIT_POSE(() -> scoringArmInitPose),
        SAMPLE_TRANSFER_POSE(() -> scoringArmSampleTransferPose),
        SCORE_FROM_FRONT(()-> scoringArmScoreFromFrontSpecimenPose),
        SCORING_ARM_SCORE_POSE(()-> scoringArmScorePose),
        SCORE_ARM_SCORE_SAMPLE(() -> scoringArmScoreSample),
        ARM_PREPARE_SAMPLE_TRANSFER_POSE(()-> scoringArmPrepareSampleTransferPose),
        INTAKE_FROM_FRONT_POSE(()-> scoringArmIntakeFromFrontPose),
        PARK(() -> scoringArmPark),
        AFTER_SCORE_FROM_FRONT(() -> afterScoreFromFrontSpecimenPose);

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

    public ScoringArm(Boolean Void) {
        servoLeft = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "L outake arm ");//1
        servoRight = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "R outake arm");//4

        servoLeft.setPosition(ScoringArmState.PARK.position.get()+0.015);
        servoRight.setPosition(1 - ScoringArmState.PARK.position.get());
    }

    public Command setPosition(double newPos) {
        return new InstantCommand(() -> {
            estimatedPose = newPos;
            servoLeft.setPosition(newPos+0.015);
            servoRight.setPosition(1 - newPos);
        },
                this);
    }


    public void setPositionVoid(double newPos) {
        estimatedPose = newPos;
        servoLeft.setPosition(newPos+0.015);
        servoRight.setPosition(1 - newPos);
    }


    public Command setPosition(ScoringArmState state) {
        return new InstantCommand(() -> {
            estimatedPose = state.position.get();
            servoLeft.setPosition(state.position.get()+0.015);
            servoRight.setPosition(1 - state.position.get());
        },
                this);
    }

    public Command setPositionWithoutRequirments(ScoringArmState state) {
        return new InstantCommand(() -> {
            estimatedPose = state.position.get();
            servoLeft.setPosition(state.position.get()+0.015);
            servoRight.setPosition(1 - state.position.get());
        });
    }

    public double getPosition(){
        return estimatedPose;
    }

}