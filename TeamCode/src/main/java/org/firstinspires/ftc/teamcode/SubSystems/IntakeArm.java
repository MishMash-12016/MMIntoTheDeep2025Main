package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class IntakeArm extends SubsystemBase {

    MMRobot mmRobot = MMRobot.getInstance();

    CuttleServo servoRight;
    CuttleServo servoLeft;

    public static enum Position {
        IN(0.6),
        OUT(0.05),
        MID(0.4);

        //TODO: check tuning
        public double intakeArmPosition;
        Position(double position) {
            this.intakeArmPosition = position;
        }
    }


    public IntakeArm() {
        servoRight = new CuttleServo(mmRobot.mmSystems.controlHub, Configuration.ARM_INTAKE_RIGHT);
        servoLeft = new CuttleServo(mmRobot.mmSystems.controlHub, Configuration.ARM_INTAKE_LEFT);
    }

    public void setPosition(double position) {
        servoLeft.setPosition(position);
        servoRight.setPosition(1-position);
    }

    public double getPosition() {
        return servoRight.getPosition();
    }

}
