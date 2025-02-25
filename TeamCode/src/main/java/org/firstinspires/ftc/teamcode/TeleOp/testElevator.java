package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

import java.util.function.DoubleSupplier;

@TeleOp
public class testElevator extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;
    boolean Specimenintake = true;

    DoubleSupplier p1 = () -> 0;
    DoubleSupplier p2 = () -> 0;

    public testElevator() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        mmSystems = robotInstance.mmSystems;

        //mmSystems.elevator.setDefaultCommand(new RunCommand(()->{},mmSystems.elevator));


        robotInstance.mmSystems.initRobotSystems();
        robotInstance.mmSystems.initDriveTrain();


        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> p1 = () -> (p1.getAsDouble()+0.01)),
                        new InstantCommand(() -> robotInstance.mmSystems.scoringArm.setPosition(p1.getAsDouble()))
                )
        );
        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> p1 = () -> (p1.getAsDouble()-0.01)),
                        new InstantCommand(() -> robotInstance.mmSystems.scoringArm.setPosition(p1.getAsDouble()))
                )
        );

        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> p2 = () -> (p2.getAsDouble()+0.01)),
                        new InstantCommand(() -> robotInstance.mmSystems.scoringEndUnitRotator.setPosition(p2.getAsDouble()))
                )
        );
        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> p2 = () -> (p2.getAsDouble()-0.01)),
                        new InstantCommand(() -> robotInstance.mmSystems.scoringEndUnitRotator.setPosition(p2.getAsDouble()))
                )
        );

//        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whileHeld(
//                ()->mmSystems.elevator.setPower(-1.0)
//        ).whenReleased(()->mmSystems.elevator.setPower(0.0));

//        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET)
//        );
//        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
//                mmSystems.elevator.ElevatorGetToZeroSensor()
//        );

    }

    @Override
    public void run() {
        super.run();

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        MMRobot.getInstance().mmSystems.elevator.updateToDashboard();
        mmSystems.driveTrain.updateTelemetry();
        telemetry.addData("arm", p1.getAsDouble());
        telemetry.addData("rot", p2.getAsDouble());

//        telemetry.addData("targertpose", mmSystems.elevator.targetPose);
//        telemetry.addData("ticks - ", mmSystems.elevator.getTicks());
//        telemetry.addData("height", mmSystems.elevator.getHeight());
//        telemetry.addData("power", mmSystems.elevator.getPower());
        telemetry.update();


    }
}
