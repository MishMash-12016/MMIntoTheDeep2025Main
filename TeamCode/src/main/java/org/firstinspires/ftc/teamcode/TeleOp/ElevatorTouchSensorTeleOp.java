package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;
@TeleOp
public class ElevatorTouchSensorTeleOp extends MMOpMode {
    MMRobot robotInstance = MMRobot.getInstance();
    //MMRobot

    public ElevatorTouchSensorTeleOp() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {
        robotInstance.mmSystems.initRobotSystems();



    }


    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        if (robotInstance.mmSystems.elevetorTouchSensor.isPressed()){
            MMRobot.getInstance().mmSystems.elevator.setTicks(0);
        }
        telemetry.addData("touched", robotInstance.mmSystems.elevetorTouchSensor.isPressed());
        telemetry.addData("encoder",robotInstance.mmSystems.elevator.getTicks());
        telemetry.addData("encoder",robotInstance.mmSystems.elevator.getHeight());
        telemetry.update();

    }


}



