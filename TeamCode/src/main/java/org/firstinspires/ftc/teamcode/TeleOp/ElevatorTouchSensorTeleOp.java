package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;
@Disabled
public class ElevatorTouchSensorTeleOp extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;
    //MMRobot

    public ElevatorTouchSensorTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {
        robotInstance = MMRobot.getInstance();
        mmSystems = robotInstance.mmSystems;


        robotInstance.mmSystems.initRobotSystems();
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                mmSystems.elevator.moveToPose(20)
        );
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                mmSystems.elevator.ElevatorGetToZero()
        );
    }


    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.addData("touched", robotInstance.mmSystems.elevator.getElevatorSwitchState());
        telemetry.addData("encoder",robotInstance.mmSystems.elevator.getTicks());
        telemetry.addData("encoder",robotInstance.mmSystems.elevator.getHeight());
        telemetry.update();

    }


}



