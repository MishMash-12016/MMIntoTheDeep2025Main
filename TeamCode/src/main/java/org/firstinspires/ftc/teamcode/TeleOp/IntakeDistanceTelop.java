package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class IntakeDistanceTelop extends MMOpMode {
    MMRobot robotInstance = MMRobot.getInstance();

    public IntakeDistanceTelop() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {
        robotInstance.mmSystems.initRobotSystems();
        new Trigger(() -> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger( //intake+intake rotater
                GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
                .whileActiveOnce(IntakeSampleCommand.prepareSampleIntake(
                        () -> MMRobot.getInstance().mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                IntakeSampleCommand.OpenClawBYsensorTest()
        );
    }


    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.addData("Dis", MMRobot.getInstance().mmSystems.intakeDistSensor.getDistance(DistanceUnit.CM));
        telemetry.update();

    }


}




