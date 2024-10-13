package org.firstinspires.ftc.teamcode.Libraries.RoadRunner.utils;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.ElevatorPIDExample.TeleOpUsageExample;

/**
 * this represents the OpMode type that has been activated.
 * <p>
 * the code that control on what to activate on which type, is located in the {@link MMRobot} class.
 * <p>
 * <b>COMPETITION:</b>
 * <p>
 * {@link Competition#TELEOP TELEOP} - Competition Teleop
 * <p>
 * {@link Competition#AUTO AUTO} - Competition Autonomous
 * <p>
 * <b>NON-COMPETITION:</b>
 * <p>
 * {@link NonCompetition#DEBUG DEBUG} - Special OpMode to control every system independently and make sure everything is working (or tuning servos).
 * <p>
 * {@link NonCompetition#EXPERIMENTING EXPERIMENTING} - This mode activates the Control hub and Expansion hub with all the attributes u might need on the {@link MMRobot}, EXCEPT the subsystems.
 * this mode should be used when u want to make a custom opmode to test a specific subsystem and make sure its working without interacting with the whole robot.
 * for example: this can be used in order to make a custom opmode for tuning the elevator's pid.
 * check the {@link TeleOpUsageExample}
 * <p>
 * {@link NonCompetition#EXPERIMENTING_NO_EXPANSION EXPERIMENTING_NO_EXPANSION} - Same thing like {@link NonCompetition#EXPERIMENTING EXPERIMENTING} just without the expansion.
 * this can be used if u don't have an expansion plugged in, and u just want to experiment with the control hub.
 */


public interface OpModeType {

    enum Competition implements OpModeType {
        TELEOP,
        AUTO
    }

    enum NonCompetition implements OpModeType {
        DEBUG,
        EXPERIMENTING,
        EXPERIMENTING_NO_EXPANSION
    }

}

