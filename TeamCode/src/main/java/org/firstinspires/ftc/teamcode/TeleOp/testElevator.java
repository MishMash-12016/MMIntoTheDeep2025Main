package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class testElevator extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;
    boolean Specimenintake = true;

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

        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whileHeld(
                ()->mmSystems.elevator.setPower(-1.0)
        ).whenReleased(()->mmSystems.elevator.setPower(0.0));

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
        telemetry.addData("targertpose", mmSystems.elevator.targetPose);
        telemetry.addData("ticks - ", mmSystems.elevator.getTicks());
        telemetry.addData("height", mmSystems.elevator.getHeight());
        telemetry.addData("power", mmSystems.elevator.getPower());
        telemetry.update();


    }
}
