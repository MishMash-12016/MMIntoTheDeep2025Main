package org.firstinspires.ftc.teamcode.CommandGroup;

import org.firstinspires.ftc.teamcode.MMRobot;

public class touchSensors {
    public static boolean getStateScoring(){
        return !MMRobot.getInstance().mmSystems.touchSensorScoring.getState();
    }
    public static boolean getStateIntake(){
        return !MMRobot.getInstance().mmSystems.touchSensorIntake.getState() || MMRobot.getInstance().mmSystems.touchSensorIntakeHigh.getState();
    }
}
