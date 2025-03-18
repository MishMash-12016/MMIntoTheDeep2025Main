package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.Supplier;

@Config
public class IntakeArm extends SubsystemBase {
    public static double intakeArmIntakeSamplePos = 0.57;
    public static double intakeArmPrepareIntakeSamplePose = 0.5;
    public static double intakeArmSpecimenIntakePose = 0.35;
    public static double intakeArmTransferSpecimenPose = 0.185;
    public static double intakeArmTransferSamplePose = 0.17;
    public static double intakeArmInitPose = 0.05;
    public static double intakeArmMidPose = 0.28;
    CuttleServo servoLeft;
    CuttleServo servoRight;

    public enum IntakeArmState {
        SAMPLE_INTAKE_POSE(()-> intakeArmIntakeSamplePos),
        PREPARE_SAMPLE_INTAKE(()-> intakeArmPrepareIntakeSamplePose),
        SPECIMEN_INTAKE(()-> intakeArmSpecimenIntakePose),
        TRANSFER_SPECIMEN_POSE(()-> intakeArmTransferSpecimenPose),
        SAMPLE_TRANSFER_POSE(()-> intakeArmTransferSamplePose),
        INIT_POSE(()-> intakeArmInitPose),
        MID_POSE(()-> intakeArmMidPose);

        public Supplier<Double> position;

        IntakeArmState(Supplier<Double> position) {
            this.position = position;
        }
    }

    public IntakeArm() {
        servoLeft = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.INTAKE_ARM_SERVO_LEFT);
        servoRight = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.INTAKE_ARM_SERVO_RIGHT);
        servoLeft.setPosition(IntakeArmState.INIT_POSE.position.get());
        servoRight.setPosition(1 - IntakeArmState.INIT_POSE.position.get());
    }
    public IntakeArm(boolean Void) {
        servoLeft = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.INTAKE_ARM_SERVO_LEFT);
        servoRight = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.INTAKE_ARM_SERVO_RIGHT);
        servoLeft.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE.position.get());
        servoRight.setPosition(1 - IntakeArmState.PREPARE_SAMPLE_INTAKE.position.get());
    }

    //tell servo intake to get to down position
    public Command setPosition(double newPos) {
        return new InstantCommand(() -> {
            servoLeft.setPosition(newPos);
            servoRight.setPosition(1 - newPos);
        },
                this);
    }

    public Command setPosition(IntakeArmState state) {
        return new InstantCommand(() -> {
            servoLeft.setPosition(state.position.get());
            servoRight.setPosition(1 - state.position.get());
        },
                this);
    }

    public void setPositionVoid(double newPos) {
        servoLeft.setPosition(newPos);
        servoRight.setPosition(1 - newPos);
    }

    public double getPosition() {
        return servoRight.getPosition();
    }
}