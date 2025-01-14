package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.roboctopi.cuttlefish.utils.Direction;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleEncoder;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.DoubleSupplier;

public class Elevator extends MMPIDSubsystem {

    //System parts:
    private final CuttleMotor motor1;
    private final CuttleMotor motor2;
    private final CuttleMotor motor3;
    private final CuttleEncoder motorEncoder;


    //constants:
    final double TICKS_PER_REV = 384.5;
    final double GEAR_RATIO = 1;
    final double LEVELS = 4;
    final double SPROCKET_PERIMETER = 6.56592;

    //PID:
    public static double kP = 0.15;
    public static double kI = 0.01;
    public static double kD = 0.002;
    public static double TOLERANCE = 0.2;
    public static double kG = 0.15;

    double ticksOffset = 0;


    public enum ElevatorState {
        LOW_BASKET(30), HIGH_BASKET(68), ELEVATOR_DOWN(5);

        public double position;

        ElevatorState(double position) {
            this.position = position;
        }
    }

    public double targetPose = 0;

    public Elevator() {
        super(kP, kI, kD, TOLERANCE);
        register();

        motor1 = new CuttleMotor(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR1);
        motor2 = new CuttleMotor(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR2);
        motor3 = new CuttleMotor(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR3);


        motorEncoder = new CuttleEncoder(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR_ENCODER, TICKS_PER_REV);

        this.motor1.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor2.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor3.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

        resetTicks();
    }

    public Command moveToPose(double setPoint) {
        return new MMPIDCommand(this, setPoint)
                .alongWith(new InstantCommand(() -> targetPose = setPoint));
    }

    public Command moveToPose(ElevatorState state) {
            return new MMPIDCommand(this, state.position)
                    .alongWith(new InstantCommand(() -> targetPose = state.position));
    }

    public Command setPowerByJoystick(DoubleSupplier power) {
        return new RunCommand(
                () -> setPower(power.getAsDouble())
                , this);
    }

    @Override
    public void setPower(Double power) {
        if(targetPose == ElevatorState.ELEVATOR_DOWN.position) {
            if (power > 0.5) {
                power = 0.48;
            }
        }
            motor1.setPower(power);
            motor2.setPower(power);
            motor3.setPower(power);
    }


    public double getTicks() {
        return motorEncoder.getCounts() + ticksOffset;
    }

    public void setTicks(double newTicks) {
        ticksOffset = newTicks - motorEncoder.getCounts();
    }

    public void resetTicks() {
        setTicks(0);
    }


    public double getHeight() {
        //getTicks-> current ticks value(current position of the encoder)
        //SPROCKET_PERIMETER -> gear diameter
        //LEVELS -> how many elevator levels there is
        return 1 * ((getTicks() / TICKS_PER_REV) * SPROCKET_PERIMETER * LEVELS / GEAR_RATIO);
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
//        FtcDashboard.getInstance().getTelemetry().addData("motorLeftPower", motorLeft.getPower());
//        FtcDashboard.getInstance().getTelemetry().addData("motorRightPower",motorRight.getPower());
        FtcDashboard.getInstance().getTelemetry().addData("height", getHeight());
        FtcDashboard.getInstance().getTelemetry().addData("target", getPidController().getSetPoint());

        FtcDashboard.getInstance().getTelemetry().update();

    }
}
