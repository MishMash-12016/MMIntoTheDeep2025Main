package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotatorYAxis;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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




//        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whileHeld(
//                ()->mmSystems.elevator.setPower(-1.0)
//        ).whenReleased(()->mmSystems.elevator.setPower(0.0));

//        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET)
//        );
//        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
//                mmSystems.elevator.ElevatorGetToZeroSensor()
//        );


//        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Trigger.RIGHT_TRIGGER).whenPressed(
//                robotInstance.mmSystems.scoringArm.setPosition(update1(true))
//        );
        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(
                      robotInstance.mmSystems.scoringArm.setPosition(0.455),
                        robotInstance.mmSystems.scoringEndUnitRotator.setPosition(0.1),
                        robotInstance.mmSystems.scoringEndUnitRotatorYAxis.setPosition(1),
                        robotInstance.mmSystems.intakeArm.setPosition(0.15),
                        robotInstance.mmSystems.scoringClawEndUnit.openScoringClaw()
                )
        );


    }

    private double p1 = 0.5;
    private final double STEP = 0.01;

    private double update1(boolean d) {
        return p1 += d ? STEP : -STEP;
    }

    @Override
    public void run() {
        super.run();

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        MMRobot.getInstance().mmSystems.elevator.updateToDashboard();
        mmSystems.driveTrain.updateTelemetry();

        telemetry.addData("targertpose", mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
//        telemetry.addData("ticks - ", mmSystems.elevator.getTicks());
//        telemetry.addData("height", mmSystems.elevator.getHeight());
//        telemetry.addData("power", mmSystems.elevator.getPower());
        telemetry.update();


    }
}
