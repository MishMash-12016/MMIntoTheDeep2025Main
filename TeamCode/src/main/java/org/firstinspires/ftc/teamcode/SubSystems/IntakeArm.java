package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.Supplier;
//0.3
@Config
public class IntakeArm extends SubsystemBase {
    public static double intakeArmIntakeSamplePos = 0.57;
    public static double intakeArmPrepareIntakeSamplePose = 0.5;
    public static double intakeArmSpecimenIntakePose = 0.3;
    public static double intakeArmTransferSamplePose = 0.08;
    public static double intakeArmInitPose = 0.05;
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

    public IntakeArm() {
        servoLeft = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.INTAKE_ARM_SERVO_LEFT);
        servoRight = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.INTAKE_ARM_SERVO_RIGHT);

        estimatedPose = IntakeArmState.INIT_POSE.position.get();
        servoLeft.setPosition(IntakeArmState.INIT_POSE.position.get());
        servoRight.setPosition(1 - IntakeArmState.INIT_POSE.position.get()+0.015);
    }
    public IntakeArm(boolean Void) {
        servoLeft = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.INTAKE_ARM_SERVO_LEFT);
        servoRight = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.INTAKE_ARM_SERVO_RIGHT);

        estimatedPose = IntakeArmState.PREPARE_SAMPLE_INTAKE.position.get();
        servoLeft.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE.position.get());
        servoRight.setPosition(1 - IntakeArmState.PREPARE_SAMPLE_INTAKE.position.get()+0.015);
    }

    public Command setPosition(double newPos) {
        estimatedPose = newPos;
        return new InstantCommand(() -> {
            servoLeft.setPosition(newPos);
            servoRight.setPosition(1 - newPos+0.015);
        },
                this);
    }

    public Command setPosition(IntakeArmState state) {
        estimatedPose = state.position.get();
        return new InstantCommand(() -> {
            servoLeft.setPosition(state.position.get());
            servoRight.setPosition(1 - state.position.get()+0.015);
        },
                this);
    }

    public Command setPositionWithoutRequirments(IntakeArmState state) {
        estimatedPose = state.position.get();
        return new InstantCommand(() -> {
            servoLeft.setPosition(state.position.get());
            servoRight.setPosition(1 - state.position.get()+0.015);
        });
    }

    public void setPositionVoid(double newPos) {
        estimatedPose = newPos;
        servoLeft.setPosition(newPos);
        servoRight.setPosition(1 - newPos+0.015);
    }

    public double getPosition() {
        return estimatedPose;
    }
}