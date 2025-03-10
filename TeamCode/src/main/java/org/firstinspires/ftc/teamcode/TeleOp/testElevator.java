package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroup.AutoSpecimensCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
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

        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                robotInstance.mmSystems.intakEndUnit.closeIntakeClaw()
        );
        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                robotInstance.mmSystems.intakEndUnit.openIntakeClaw()
        );




//        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whileHeld(
//                ()->mmSystems.elevator.setPower(-1.0)
//        ).whenReleased(()->mmSystems.elevator.setPower(0.0));

//        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                mmSystems.elevator.moveToPose(Elevator.ElevatorState.LOW_BASKET)
//        );














//        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
//                mmSystems.elevator.ElevatorGetToZeroSensor()
//        );


//        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Trigger.RIGHT_TRIGGER).whenPressed(
//                robotInstance.mmSystems.scoringArm.setPosition(update1(true))
//        );
//        mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER) // sample
//                .whenPressed(
//                        IntakeSampleCommand.prepareSampleIntake(
//                                () -> mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get(),
//                                () -> mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get()
//                        )
//                );
//
//        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
//                IntakeSampleCommand.SampleIntake()
//        );
//
//        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new SequentialCommandGroup(
//                        robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.MID_POSE),
//                        new WaitCommand(100),
//                        robotInstance.mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SAMPLE_POSE),
//                        new WaitCommand(100),
//                        robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SAMPLE_TRANSFER_POSE),
//                        robotInstance.mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SAMPLE_TRANSFER_POSE),
//                        robotInstance.mmSystems.scoringClawEndUnit.openScoringClaw(),
//                        new WaitCommand(300),
//                        robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SAMPLE_TRANSFER_POSE),
//                        robotInstance.mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.HOLD_POSE_SPECIMEN)
//                        )
//        );

//        robotInstance.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new SequentialCommandGroup(
//                        robotInstance.mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORE_SPECIMEN_POSE),
//                        robotInstance.mmSystems.scoringEndUnitRotatorYAxis.setPosition(ScoringEndUnitRotatorYAxis.ScoringRotatorYAxisState.SCORE_POSE),
//                      new WaitCommand(500),
//                      robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORE_SPECIMEN)
//
//
//                )
//        );
//        robotInstance.mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
//                robotInstance.mmSystems.scoringEndUnitRotatorYAxis.setPosition(.16)
//        );
//        robotInstance.mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                robotInstance.mmSystems.scoringEndUnitRotatorYAxis.setPosition(.17)
//        );
//        robotInstance.mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                robotInstance.mmSystems.scoringEndUnitRotatorYAxis.setPosition(.18)//
//        );
//
//        robotInstance.mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
//                robotInstance.mmSystems.scoringArm.setPosition(0)
//        );
//        robotInstance.mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
//                robotInstance.mmSystems.scoringArm.setPosition(.25)//
//        );
//        robotInstance.mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                robotInstance.mmSystems.scoringArm.setPosition(.75)
//        );
//        robotInstance.mmSystems.gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
//                robotInstance.mmSystems.scoringArm.setPosition(.8)
//        );

    }

//    private double p1 = 0.5;
//    private final double STEP = 0.01;
//
//    private double update1(boolean d) {
//        return p1 += d ? STEP : -STEP;
//    }

    @Override
    public void run() {
        super.run();

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        MMRobot.getInstance().mmSystems.elevator.updateToDashboard();
        mmSystems.driveTrain.updateTelemetry();

        telemetry.addData("targertpose", mmSystems.gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
//        telemetry.addData("ticks - ", mmSystems.elevator.getTicks());
//        telemetry.addData("height", mmSystems.elevator.getHeight());
//        telemetry.addData("power", mmSystems.elevator.getPower());
        telemetry.update();


    }
}
