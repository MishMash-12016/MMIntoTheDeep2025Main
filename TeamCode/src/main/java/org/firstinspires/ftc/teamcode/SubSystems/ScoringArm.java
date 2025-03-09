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
    private static final double scoringArmMidePose = 0.3;
    private static final double scoringArmRestPose = 0.46;
    private static final double scoringArmSpecimenTransferPose = 0.62;
    private static final double scoringArmSampleTransferPose = 0.44;
    private static final double scoringArmInitPose = 0.4;
    private static final double scoringArmSpecimenScorePose = 0.7;
    private static final double scoringArmSampleScorePose = 0.3;
    private static final double scoringArmSamplePrepareScorePose = 0.33;
    private static final double scoringArmIntakeFromBackPose = 0.21;
    private static final double scoringArmAfterScoreSpecimenPose = 0.85;

    public enum ScoringArmState {
        MID_POSE(() -> scoringArmMidePose),
        REST_POSE(() -> scoringArmRestPose),
        SPECIMEN_TRANSFER_POSE(() -> scoringArmSpecimenTransferPose),
        SAMPLE_TRANSFER_POSE(() -> scoringArmSampleTransferPose),
        INIT_POSE(() -> scoringArmInitPose),
        SCORE_SPECIMEN(() -> scoringArmSpecimenScorePose),
        SCORE_SAMPLE(() -> scoringArmSampleScorePose),
        PREPARE_SCORE_SAMPLE(() -> scoringArmSamplePrepareScorePose),
        AFTER_SCORE_POSE(()-> scoringArmAfterScoreSpecimenPose),
        INTAKE_FROM_BACK_POSE(()-> scoringArmIntakeFromBackPose);
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
        servoLeft.setPosition(ScoringArmState.INIT_POSE.position.get());
        servoRight.setPosition(1 - ScoringArmState.INIT_POSE.position.get());
    }

    //Tell arm to get to position
    public Command setleftPosition(double newPos) {
        return new InstantCommand(() -> {
            servoLeft.setPosition(newPos);
        },

                this);
    }

    public Command setPosition(double newPos) {
        return new InstantCommand(() -> {
            servoLeft.setPosition(newPos);
            servoRight.setPosition(1 - newPos);
        },
                this);
    }


    public Command setPosition(ScoringArmState state) {
        return new InstantCommand(() -> {
            servoLeft.setPosition(state.position.get());
            servoRight.setPosition(1 - state.position.get());
        },
                this);
    }

    public double getPosition() {
        return servoRight.getPosition();
    }
}