package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.Commands.MMToggleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Commands.MMToggleCommand2;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Commands.RotateTurretByPid;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Subsystems.ShooterIntake;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Subsystems.ShooterTurret;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.SubsystemStructure.IMMPositionSubsystem;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.SubsystemStructure.IMMPowerSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;


/**
 * this class is an example of how to answer to mechanics' request quickly
 */
@Disabled
@TeleOp(name = "Nimrod's Shooter")
public class ShooterTestAlsoAnExampleOnMMToggleCommand extends MMOpMode {

    public ShooterTestAlsoAnExampleOnMMToggleCommand() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    MMRobot mmRobot = MMRobot.getInstance();

    @Override
    public void onInit() {

        mmRobot.mmSystems.shooter = new Shooter();
        mmRobot.mmSystems.shooterIntake = new ShooterIntake();
        mmRobot.mmSystems.shooterTurret = new ShooterTurret();


        //Turret

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileActiveOnce(
                new MMToggleCommand2<>(
                        mmRobot.mmSystems.shooterTurret,
                        0.5, 0.
                )
        );

        //this is in case tuning is requested for example:

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new MMPIDCommand(mmRobot.mmSystems.shooterTurret, 30)
        );

        //in case u want to change smth in the logic,
        //u can use a new class command that inherits from MMPIDCommand and overrides the methods.
        //(for now it does the same as u can see)
        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new RotateTurretByPid(0)
        );

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new MMPIDCommand(mmRobot.mmSystems.shooterTurret, -30)
        );


        //Shooter and Intake

        //the shooter can be addressed as both a power and position subsystem, here are the 2 examples:
        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).toggleWhenActive(
                new MMToggleCommand2<>(
                        (IMMPowerSubsystem<Double>) mmRobot.mmSystems.shooter,
                        1., 0.,
                        mmRobot.mmSystems.shooter
                )
        );
        //notice that here, if i used requirements for this command, i wouldn't be able to activate A and B together.
        //because they are both requiring the same subsystem.
        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).toggleWhenActive(
                new MMToggleCommand2<>(
                        (IMMPositionSubsystem<Double>) mmRobot.mmSystems.shooter,
                        0.2, 0.
                )
        );

        //this is an example of how to pass a consumer.
        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).toggleWhenActive(
                new MMToggleCommand<>(
                        (x) -> {
                            mmRobot.mmSystems.shooterIntake.setMotorPower(x);
                            mmRobot.mmSystems.shooterIntake.setServoPower(x);
                        },
                        1., 0.,
                        mmRobot.mmSystems.shooterIntake
                )

        );

        //this is an example of how to pass a consumer of 1 method.
        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).toggleWhenActive(
                new MMToggleCommand<>(
                        mmRobot.mmSystems.shooterIntake::setMotorPower,
                        1., 0.,
                        mmRobot.mmSystems.shooterIntake
                )
        );

    }
}
