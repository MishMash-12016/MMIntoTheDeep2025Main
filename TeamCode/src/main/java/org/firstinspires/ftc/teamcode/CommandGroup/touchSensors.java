package org.firstinspires.ftc.teamcode.CommandGroup;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;

public class touchSensors {
    public static boolean getStateBumper(){
        return !MMRobot.getInstance().mmSystems.touchSensorBumper.getState();
    }
    public static boolean getStateArm(){
        return !MMRobot.getInstance().mmSystems.touchSensorArm.getState();
    }
}
