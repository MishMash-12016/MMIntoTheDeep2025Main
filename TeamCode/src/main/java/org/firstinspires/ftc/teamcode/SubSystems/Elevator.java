package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.roboctopi.cuttlefish.utils.Direction;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleDigital;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleEncoder;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommandForever;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.Supplier;

@Config
public class Elevator extends MMPIDSubsystem {

    //System parts:
    private final CuttleMotor motor1;
    private final CuttleMotor motor2;
    private final CuttleMotor motor3;
    private final CuttleMotor motor4;
    public final CuttleEncoder motorEncoder;
    public final CuttleDigital elevatorSwitch;


    //constants:
    final double TICKS_PER_REV = 145.1;
    final double GEAR_RATIO = 1.25 * 40/22;
    final double LEVELS = 1;
    final double SPROCKET_PERIMETER = Math.PI*3.82;

    //PID:
    public static double kP = 0.19;
    public static double kI = 0;
    public static double kD = 0.0001;

    public static double TOLERANCE = .3;
    public static double kG = 0.0;

    public double ticksOffset = 0;

    private static final double maxHeight = 105; //TODO:find new max height

    public static double elevatorHighBasket = 43;
    public static double elevatorDown = 0;
    public static double elevatorClimbLow = 23;
    public static double elevatorClimb = 12;

    public enum ElevatorState {


        HIGH_BASKET(() -> elevatorHighBasket),
        ELEVATOR_DOWN(() -> elevatorDown),
        ELEVATOR_LOW_CHAMBER(() -> elevatorClimbLow),
        ELEVATOR_CLIMB(() -> elevatorClimb);

        public Supplier<Double> position;

        ElevatorState(Supplier<Double> position) {
            this.position = position;
        }
    }

    public double targetPose = 0;
    MMPIDCommandForever PID;
    public Elevator() {
        super(kP, kI, kD, TOLERANCE);

        register();

        motor1 = new CuttleMotor(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR1);
        motor2 = new CuttleMotor(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR2);
        motor3 = new CuttleMotor(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR3);
        motor4 = new CuttleMotor(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR4);

        elevatorSwitch = new CuttleDigital(MMRobot.getInstance().mmSystems.expansionHub, Configuration.elevatorTouchSensor);

        this.motor1.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor2.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor3.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor4.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1.setDirection(Direction.REVERSE);
        motor2.setDirection(Direction.REVERSE);
        motor3.setDirection(Direction.REVERSE);
        motor4.setDirection(Direction.REVERSE);

        motorEncoder = new CuttleEncoder(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR_ENCODER, TICKS_PER_REV);
        resetTicks();

        PID = new MMPIDCommandForever(this);
        setDefaultCommand(PID);
    }

    public Command moveToPose(double setPoint) {
        return new MMPIDCommand(this, setPoint)
                .alongWith(new InstantCommand(() -> targetPose = setPoint));
    }

    public Command moveToPose(ElevatorState state) {
        return new MMPIDCommand(this, state.position.get())
                .alongWith(new InstantCommand(() -> targetPose = state.position.get()));
    }

    public boolean getElevatorSwitchState() {
        return !elevatorSwitch.getState();
    }

    public Command ElevatorGetToZeroSensor() {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.elevator.getElevatorSwitchState()),
                        new SequentialCommandGroup(
                                moveToPose(ElevatorState.ELEVATOR_DOWN),
                                new RunCommand(() -> setPower(-0.3))
                        )

                ).withTimeout(3000),//TODO:smaller number

                new WaitCommand(200),
                new InstantCommand(() -> setTicks(0)),
                new InstantCommand(() -> setPower(0.0))
        );
    }

    @Override
    public void setPower(Double power) {
        if (targetPose > maxHeight) {
            power = 0.0;
        }
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(power);
    }


    public double getTicks() {
        return motorEncoder.getCounts() * -1 + ticksOffset;
    }

    public double getTicksOffset() {
        return ticksOffset;
    }

    public void setTicks(double newTicks) {
        ticksOffset = newTicks - motorEncoder.getCounts() * -1;
    }

    public void resetTicks() {
        setTicks(0);
    }

    public double getHeight() {
        //getTicks-> current ticks value(current position of the encoder)
        //SPROCKET_PERIMETER -> gear diameter
        //LEVELS -> how many elevator levels there is
        return (getTicks() / TICKS_PER_REV) / GEAR_RATIO * SPROCKET_PERIMETER * LEVELS;
    }

    @Override
    public double getCurrentValue() {
        return getHeight();
    }

    @Override
    public double getFeedForwardPower() {
        return kG * Math.signum(targetPose - getHeight());
    }

    @Override
    public void stop() {
        setPower(kG);
    }

    @Override
    public void periodic() {
        updateToDashboard();
    }

    public void updateToDashboard() {
        FtcDashboard.getInstance().getTelemetry().addData("height", getHeight());
        FtcDashboard.getInstance().getTelemetry().addData("target", getPidController().atSetpoint());
        FtcDashboard.getInstance().getTelemetry().addData("elevator power", motor1.getPower());

        FtcDashboard.getInstance().getTelemetry().update();

    }

    public double getPower() {
        return motor1.getPower();
    }

    public void disablePID() {
        PID.subsystem.doPid = false;
    }
}
