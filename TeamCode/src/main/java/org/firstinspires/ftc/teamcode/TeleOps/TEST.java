package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleEncoder;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.utils.OpModeType;

@TeleOp
public class TEST extends MMTeleOp {
    public TEST() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }
    CuttleMotor motor;
    CuttleEncoder encoder;

    @Override
    public void onInit() {
        motor = new CuttleMotor(MMRobot.getInstance().mmSystems.expansionHub, 3);
        encoder = new CuttleEncoder(MMRobot.getInstance().mmSystems.expansionHub,3,1);

    }

    @Override
    public void run() {
        super.run();
        motor.setPower(gamepad1.left_trigger);
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.addData("meow",encoder.getCounts());
       telemetry.update();

    }
}
