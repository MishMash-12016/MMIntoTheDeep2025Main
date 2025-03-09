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
    public static double scoreingArmMidePose = 0.3;
    public static double scoringArmRestPose = 0.46;
    public static double scoringArmSpecimenTransferPose = 0.6;
    public static double scoringArmSampleTransferPose = 0.44
    public static double scoringArmInitPose = 0.05;
    public static double scoringArmSpecimenScorePose = 0.8;
    public static double scoringArmSampleScorePose = 0.3;
    public static double scoringArmSamplePrepareScorePose = 0.33;
    public static double intakeFromBackPose = 0.21;
    public static double afterScoreSpecimenPose = 0.23;

    public enum ScoringArmState {
        MID_POSE(() -> scoreingArmMidePose),
        REST_POSE(() -> scoringArmRestPose),
        SPECIMEN_TRANSFER_POSE(() -> scoringArmSpecimenTransferPose),
        SAMPLE_TRANSFER_POSE(() -> scoringArmSampleTransferPose),
        INIT_POSE(() -> scoringArmInitPose),
        SCORE_SPECIMEN(() -> scoringArmSpecimenScorePose),
        SCORE_SAMPLE(() -> scoringArmSampleScorePose),
        PREPARE_SCORE_SAMPLE(() -> scoringArmSamplePrepareScorePose),
        AFTER_SCORE_POSE(()-> afterScoreSpecimenPose),
        INTAKE_FROM_BACK_POSE(()-> intakeFromBackPose);
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