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
    public static double intakeArmIntakePos = 0.57;
    public static double intakeArmPrepareSampleIntakePos = 0.5;
    public static double intakeArmSpecimenIntakePos = 0.33;
    public static double intakeArmMidIntakeSpecimenPos = 0.3;
    public static double intakeArmTransferSpecimenPos = 0.185;
    public static double intakeArmTransferSamplePos = 0.18;
    public static double intakeArmInitPos = 0.05;
    CuttleServo servoLeft;
    CuttleServo servoRight;

    public enum IntakeArmState {
        INTAKE_POSE(()-> intakeArmIntakePos),
        PREPARE_SAMPLE_INTAKE(()-> intakeArmPrepareSampleIntakePos),
        SPECIMEN_INTAKE(()-> intakeArmSpecimenIntakePos),
        MID_INTAKE_SPECIMEN(()-> intakeArmMidIntakeSpecimenPos),

        TRANSFER_POSE(()-> intakeArmTransferSpecimenPos),
        SAMPLE_TRANSFER_POSE(()-> intakeArmTransferSamplePos),
        INIT_POSE(()-> intakeArmInitPos);

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
}