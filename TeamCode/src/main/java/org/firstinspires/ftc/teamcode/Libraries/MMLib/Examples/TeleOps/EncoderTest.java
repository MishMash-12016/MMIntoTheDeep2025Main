package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Disabled
@TeleOp
public class EncoderTest extends MMOpMode {

    public EncoderTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    DcMotorEx motorEx;

    @Override
    public void onInit() {

        //notice that this is without the cuttlefish library
        motorEx = hardwareMap.get(DcMotorEx.class, "shooter");

        addRunnableOnInit(
                () -> motorEx.setPower(0.5)
        );

    }

    @Override
    public void run() {
        super.run();
        //if u were to use CuttleMotor, there shouldve been a pullBulkData call. (read the super method for more details)
        telemetry.addData("encoder value", motorEx.getCurrentPosition());
        telemetry.update();
    }
}
