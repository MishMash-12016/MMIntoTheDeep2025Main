package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimansCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSpecimanCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.DriveTrain.Commands.ResetFieldOrientedCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@TeleOp
public class ManualDrive extends MMOpMode {
    MMRobot robotInstance;
    MMSystems mmSystems;
    boolean mode = false;

    public ManualDrive() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        mmSystems = robotInstance.mmSystems;


        robotInstance.mmSystems.initRobotSystems();
        robotInstance.mmSystems.initDriveTrain();
        new Trigger(() -> mmSystems.gamepadEx1.getTrigger( // prepare intake
                GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
                .whenActive(
                        new ConditionalCommand(IntakeSampleCommand.prepareSampleIntake(()-> mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get()),
                                IntakeSpecimansCommand.PrepareSpecimanIntake(),
                                () -> mode));

        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed( //intake - A/ X (down button)
                new ConditionalCommand(IntakeSampleCommand.SampleIntake(),
                        IntakeSpecimansCommand.SpecimenIntake(),
                        () -> mode)

        );

        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ConditionalCommand(ScoringSampleCommand.ScoreHighSample(),
                        ScoringSpecimanCommand.SpecimanScore(),
                        () -> mode)); //score - triangle
                ;
        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                ScoringSampleCommand.ScoreLowSample() //O or right low basket
        );


}

@Override
public void run() {
    super.run();
    if (mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.BACK).get()) {
        mode = true;
    }

    MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
    mmSystems.elevator.updateToDashboard();
    mmSystems.driveTrain.updateTelemetry();
    telemetry.addData("targertpose", mmSystems.elevator.targetPose);
    telemetry.addData("height", mmSystems.elevator.getHeight());
    telemetry.update();

}
}
