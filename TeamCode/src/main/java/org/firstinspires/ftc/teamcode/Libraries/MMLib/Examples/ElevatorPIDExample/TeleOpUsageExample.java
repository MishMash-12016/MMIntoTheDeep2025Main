package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.ElevatorPIDExample;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMTuningFFCommand;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

public class TeleOpUsageExample extends MMOpMode {


    public TeleOpUsageExample() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    /**
     * the commands here are using the standard {@link MMPIDCommand}
     * if u have additional logic to add to ur elevator command,
     * u can create a new class command and extend from {@link MMPIDCommand} and override the methods to ur liking.<p>
     * u can also use anonymous class if u hate organized code.
     */
    @Override
    public void onInit() {

        //initializing the elevator
        MMRobot.getInstance().mmSystems.initExampleElevator();

        //creating custom command-bindings for tuning: (using dashboard)

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new MMPIDCommand( //up to 15cm
                        MMRobot.getInstance().mmSystems.exampleElevator,
                        15
                )
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new MMPIDCommand( //back to 0
                        MMRobot.getInstance().mmSystems.exampleElevator,
                        0
                )
        );

        //creating custom FF tuning command: (using telemetry)

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new MMTuningFFCommand( //minimal power to reach 1cm
                        MMRobot.getInstance().mmSystems.exampleElevator,
                        1
                )
        );


    }
}
