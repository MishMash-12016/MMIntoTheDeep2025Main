package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.function.Supplier;

@Config
public class IntakEndUnit extends SubsystemBase {
    public static double IntakeClawClosePos = 0.1;
    public static double IntakeClawOpenPos = 0.6;


    Servo clawIntakeServo;

    public enum IntakeClawState {
        OPEN(()-> IntakeClawOpenPos), CLOSE(()-> IntakeClawClosePos);
        public Supplier<Double> position;

        IntakeClawState(Supplier<Double> position) {
            this.position = position;
        }
    }

    // claw close or open
    public IntakEndUnit() {
        clawIntakeServo = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "intake claw");//4
        // clawIntakeServo = new CuttleServo(MMRobot.getInstance().mmSystems.controlHub, Configuration.CLAW_INTAKE_SERVO);


    }

    public Command openIntakeClaw() {
        return new InstantCommand(() -> clawIntakeServo.setPosition(IntakeClawState.OPEN.position.get()), this);
    }
    public Command closeIntakeClaw() {
        return new InstantCommand(() -> clawIntakeServo.setPosition(IntakeClawState.CLOSE.position.get()), this);
    }

    public Command setPose(double pose) {
        return new InstantCommand(() -> clawIntakeServo.setPosition(pose), this);
    }

    public Command setPoseWithoutRequirments(double pose) {
        return new InstantCommand(() -> clawIntakeServo.setPosition(pose));
    }

    @Override
    public void periodic() {
        FtcDashboard.getInstance().getTelemetry().addData("clawPose", clawIntakeServo.getPosition());
    }
}
