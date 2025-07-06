package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.Supplier;

//0.3
@Config
public class
IntakeArm extends SubsystemBase {
    public static double intakeArmIntakeSamplePos = 0.63;
    public static double intakeArmPrepareIntakeSamplePose = 0.52;
    public static double intakeArmSpecimenIntakePose = 0.33;
    public static double intakeArmTransferSamplePose = 0.08;
    public static double intakeArmInitPose = 0.08;
    public double estimatedPose;
    CuttleServo servoLeft;
    CuttleServo servoRight;

    public enum IntakeArmState {
        SAMPLE_INTAKE_POSE(()-> intakeArmIntakeSamplePos),
        PREPARE_SAMPLE_INTAKE(()-> intakeArmPrepareIntakeSamplePose),
        SPECIMEN_INTAKE(()-> intakeArmSpecimenIntakePose),
        SAMPLE_TRANSFER_POSE(()-> intakeArmTransferSamplePose),
        INIT_POSE(()-> intakeArmInitPose);

        public Supplier<Double> position;

        IntakeArmState(Supplier<Double> position) {
            this.position = position;
        }
    }
    Servo intakeLeft;
    Servo intakeRight;
    public IntakeArm() {

        intakeLeft = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "L intake arm ");//5
        intakeRight = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "R intake arm"); //2

        estimatedPose = IntakeArmState.INIT_POSE.position.get();
        intakeLeft.setPosition(IntakeArmState.INIT_POSE.position.get());
        intakeRight.setPosition(1 - IntakeArmState.INIT_POSE.position.get()+0.015);
    }
    public IntakeArm(boolean Void) {
        intakeLeft = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "L intake arm ");//5
        intakeRight = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "R intake arm"); //2

        estimatedPose = IntakeArmState.PREPARE_SAMPLE_INTAKE.position.get();
        intakeLeft.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE.position.get());
        intakeRight.setPosition(1 - IntakeArmState.PREPARE_SAMPLE_INTAKE.position.get()+0.015);
    }

    public IntakeArm(int dontMove) {
        intakeLeft = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "L intake arm ");//5
        intakeRight = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "R intake arm"); //2
 }

    public Command setPosition(double newPos) {
        estimatedPose = newPos;
        return new InstantCommand(() -> {
            intakeLeft.setPosition(newPos);
            intakeRight.setPosition(1 - newPos+0.015);
        },
                this);
    }

    public Command setPosition(IntakeArmState state) {
        estimatedPose = state.position.get();
        return new InstantCommand(() -> {
            intakeLeft.setPosition(state.position.get());
            intakeRight.setPosition(1 - state.position.get()+0.015);
        },
                this);
    }

    public Command setPositionWithoutRequirments(IntakeArmState state) {
        estimatedPose = state.position.get();
        return new InstantCommand(() -> {
            intakeLeft.setPosition(state.position.get());
            intakeRight.setPosition(1 - state.position.get()+0.015);
        });
    }

    public void setPositionVoid(double newPos) {
        estimatedPose = newPos;
        intakeLeft.setPosition(newPos);
        intakeRight.setPosition(1 - newPos+0.015);
    }

    public double getPosition() {
        return estimatedPose;
    }
}