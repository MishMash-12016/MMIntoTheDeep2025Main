package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.TeleOps;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.Commands.MMDeferredCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

import java.util.Arrays;
import java.util.HashSet;
import java.util.stream.Collectors;

public class DeferredCommandTest extends MMTeleOp {

    public DeferredCommandTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new MMDeferredCommand(
                        InstantCommand::new, //example command supplier
                        new HashSet<Subsystem>() {{ //command requirements
                            add(MMRobot.getInstance().mmSystems.shooter);
                            add(MMRobot.getInstance().mmSystems.shooterIntake);
                        }}
                )
        );


        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new MMDeferredCommand(
                        InstantCommand::new,
                        Arrays.stream(
                                new Subsystem[]{
                                        MMRobot.getInstance().mmSystems.shooter,
                                        MMRobot.getInstance().mmSystems.shooterIntake
                                }
                        ).collect(Collectors.toSet())
                )
        );

    }
}
