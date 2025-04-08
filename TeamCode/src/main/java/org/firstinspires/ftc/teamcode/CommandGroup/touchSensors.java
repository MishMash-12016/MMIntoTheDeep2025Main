package org.firstinspires.ftc.teamcode.CommandGroup;

import org.firstinspires.ftc.teamcode.MMRobot;

public class touchSensors {
    public static boolean getStateBack(){
        return !MMRobot.getInstance().mmSystems.touchSensorBumper.getState();
    }
    public static boolean getStateFront(){
        return MMRobot.getInstance().mmSystems.touchSensorArm.getState();
    }
}
