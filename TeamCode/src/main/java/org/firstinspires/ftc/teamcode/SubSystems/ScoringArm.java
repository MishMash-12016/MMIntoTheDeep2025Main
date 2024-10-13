package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.utils.Configuration;

public class ScoringArm extends SubsystemBase {

    MMRobot robot = MMRobot.getInstance();

    public enum Position {
        SCORING(0.52),
        IN(0);

        public double scoringArmPosition;

        Position(double position) {
            this.scoringArmPosition = position;
        }
    }


    CuttleServo servoRight;
    CuttleServo servoLeft;

    public ScoringArm() {
        servoRight = new CuttleServo(robot.mmSystems.expansionHub, Configuration.SCORING_ARM_RIGHT);
        servoLeft = new CuttleServo(robot.mmSystems.expansionHub, Configuration.SCORING_ARM_LEFT);
    }

    public void setPosition(double position) {
        servoRight.setPosition(position);
        servoLeft.setPosition(position);
    }

    public double getPosition() {
        return servoRight.getPosition();
    }


}
