package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.utils.OpModeType;

/**
 * this class should represent ur robot singleton.
 * <p>
 * this repo was built in the 2023-2024 CENTERSTAGE OffSeason as Nevo's final project.
 * this is an EXTENSION of FTCLib and CuttlefishFTCBridge by roboctopi.
 * you're welcome to (and expected to) change everything to your liking.
 * this was all built as a template to help reduce work in the upcoming seasons,
 * but this under no circumstances should LIMIT your creativity or ability to grow and learn,
 * remember, you are your only limit.
 * </p>
 * good luck <3
 */
public class MMRobot extends Robot {

    /**
     * the robot instance
     */
    private static MMRobot instance;

    /**
     * the get method for the singleton
     * @return the robot instance
     */
    public static synchronized MMRobot getInstance() {
        if (instance == null) {
            instance = new MMRobot();
        }
        return instance;
    }

    public synchronized void resetRobot(){
        instance = null;
    }

    //Systems
    public MMSystems mmSystems;

    public void init(OpModeType type, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        mmSystems = new MMSystems(type, hardwareMap, gamepad1, gamepad2, telemetry);
        initializeSystems(type);
    }

    public void init(OpModeType type, AllianceColor allianceColor, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        init(type, hardwareMap, gamepad1, gamepad2, telemetry);
        mmSystems.allianceColor = allianceColor;
    }

    public void init(OpModeType type, AllianceColor allianceColor, AllianceSide robotSide, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        init(type, allianceColor, hardwareMap, gamepad1, gamepad2, telemetry);
        mmSystems.robotSide = robotSide;
    }


    /**
     * this initializes your subsystems.
     * <p>
     * if experimenting, then this does nothing.
     * @param type the {@link OpModeType} chosen
     */
    private void initializeSystems(OpModeType type) {
        if(type == OpModeType.Competition.TELEOP) {
            initTele();
        } else if (type == OpModeType.Competition.AUTO) {
            initAuto();
        } else if(type == OpModeType.NonCompetition.DEBUG) {
            initDebug();
        }
    }

    /**
     * initialize subsystems required for teleop
     */
    private void initTele() {
        //for example:
        MMRobot.getInstance().mmSystems.initShooterPID();
    }

    /**
     * initialize subsystems required for auto
     */
    private void initAuto() {
       //TODO a day before comp
    }

    /**
     * initialize subsystems required for debug mode
     */
    private void initDebug() {
        //TODO when mechanics tells u "hey can u make this servo move in weird ahh ways at 0.000001 speed"
    }

}
