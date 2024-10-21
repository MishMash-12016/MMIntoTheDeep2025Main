package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.ElevatorPIDExample;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleEncoder;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class ExampleElevator extends MMPIDSubsystem {

    //hardware
    private final CuttleMotor motorLeft;
    private final CuttleMotor motorRight;
    private final CuttleEncoder encoder;

    //control
    private static final double kP = 0;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double tolerance = 0;

    //constants
    private final double TICKS_PER_REV = 384.5;
    private final double SPROCKET_PITCH_RADIUS = 1.05;
    private final int ELEVATOR_LEVELS = 3;

    public ExampleElevator() {
        super(kP, kI, kD, tolerance);
        motorLeft = new CuttleMotor(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR_LEFT);
        motorRight = new CuttleMotor(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR_RIGHT);
        encoder = new CuttleEncoder(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR_ENCODER, TICKS_PER_REV);
    }

    @Override
    public void setPower(Double power) {
        motorRight.setPower(power);
        motorLeft.setPower(power);
    }

    /**
     * get the current value (height) of the elevator.
     * <p>
     * note that it's different in every elevator,
     * this is something you are going to have to find urself. </p>
     * the method below, represents the CENTERSTAGE elevator's method.
     * @return the height in cm.
     */
    @Override
    public double getCurrentValue() {
        double revs = encoder.getCounts() / TICKS_PER_REV;
        double chainLengthPerRev = 2 * Math.PI * SPROCKET_PITCH_RADIUS;
        return revs * chainLengthPerRev * ELEVATOR_LEVELS;
    }

    @Override
    public void stop() {
        setPower((double) 0);
    }
}
