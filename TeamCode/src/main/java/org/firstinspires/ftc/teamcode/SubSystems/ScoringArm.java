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
    public static double scoringarmrestPos = 0.7;
    public static double scoringarmtransferPose = 0.605;
    public static double scoringarmsampleTransferPose = 0.62;
    public static double scoringarmparkAuto = 0.34;
    public static double scoringarminitPose = 0.6;
    public static double scoringarmprepareTransferPose = 0.63;
    public static double scoringarmmidPose = 0.63;
    public static double scoringarmscoreSpecimenPose = 0.46;
    public static double scoringarmscoreSamplePose = 0.26;
    public static double scoringarmprepareScoreSamplePose = 0.32;
    public static double intakeFormBackPos = 0.21;

    public enum ScoringArmState {
        REST_POSE(() -> scoringarmrestPos),
        TRANSFER_POSE(() -> scoringarmtransferPose),
        SAMPLE_TRANSFER_POSE(() -> scoringarmsampleTransferPose),
        PARK_AUTO(() -> scoringarmparkAuto),
        INIT_POSE(() -> scoringarminitPose),
        PREPARE_TRANSFER(() -> scoringarmprepareTransferPose),
        MID_POSE(() -> scoringarmmidPose),
        SCORE_SPECIMEN(() -> scoringarmscoreSpecimenPose),
        SCORE_SAMPLE(() -> scoringarmscoreSamplePose),
        PREPARE_SCORE_SAMPLE(() -> scoringarmprepareScoreSamplePose),
        INTAKE_FROM_BACK_POSE(()->intakeFormBackPos);
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