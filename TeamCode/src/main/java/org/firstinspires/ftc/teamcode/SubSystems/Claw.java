package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.utils.Configuration;

public class Claw extends SubsystemBase {

    MMRobot robot = MMRobot.getInstance();

    CuttleServo servo;

    public enum State {
        CLOSE(1), OPEN(0.3);
        public double position;

        State(double position){
            this.position = position;
        }
    }


    public Claw(){
        servo = new CuttleServo(robot.mmSystems.expansionHub,Configuration.CLAW_SCORING);
    }

    public void setPosition(double position){
        servo.setPosition(position);
    }

    public double getPosition(){
        return servo.getPosition();
    }

}
