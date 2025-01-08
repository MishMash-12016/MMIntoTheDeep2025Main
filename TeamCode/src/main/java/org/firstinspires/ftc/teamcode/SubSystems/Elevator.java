package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
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
    private final double TICKS_PER_REV = 537.7;
    private final double GEAR_RATIO = 1;
    private final double LEVELS = 4;
    private final double SPROCKET_PERIMETER = 12.9;

    //PID:
    public static final double kP = 0.15;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0.3;

public static final double TOLERANCE = 4;

    double ticksOffset = 0;

    public final static double LOW_BASKET = 80;
    public final static double HIGH_BASKET = 140;
    public final static double elevatorDown = 1;
    public final static double elevatorWallHeight = 3;
    public final static double highChamber = 4;
    public final static double highChamberScorePose = 4;

    public double targetPose = 0;

    public Elevator() {
        super(kP, kI, kD, TOLERANCE);
    register();

        motor1 = new CuttleMotor(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR1);
        motor2 = new CuttleMotor(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR2);
        motor3 = new CuttleMotor(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR3);


        motorEncoder = new CuttleEncoder(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR_ENCODER, TICKS_PER_REV);

//        this.motorLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.motorRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

        resetTicks();
    }

    public Command moveToPose(double setPoint) {
        return new MMPIDCommand(this, setPoint)
                .alongWith(new InstantCommand(()->targetPose = setPoint));
    }

    public Command setPowerByJoystick(DoubleSupplier power) {
        return new RunCommand(
                () -> setPower(power.getAsDouble())
                , this);
    }

    @Override
    public void setPower(Double power){
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
        setPower(0.0);
    }
    @Override
    public void periodic() {
        updateToDashboard();
    }

    public void updateToDashboard(){
//        FtcDashboard.getInstance().getTelemetry().addData("motorLeftPower", motorLeft.getPower());
//        FtcDashboard.getInstance().getTelemetry().addData("motorRightPower",motorRight.getPower());
        FtcDashboard.getInstance().getTelemetry().addData("height",getHeight());
        FtcDashboard.getInstance().getTelemetry().addData("target", getPidController().getSetPoint());
        FtcDashboard.getInstance().getTelemetry().update();

    }
}
