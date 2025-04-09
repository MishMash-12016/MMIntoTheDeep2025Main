package org.firstinspires.ftc.teamcode.Libraries.MMLib.PID;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.SubsystemStructure.MMPowerSubsystem;
import org.firstinspires.ftc.teamcode.utils.SQPIDController;

public abstract class MMPIDSubsystem extends MMPowerSubsystem<Double> {
    public boolean doPid = true;

    private final SQPIDController pidController;

    /**
     * default constructor to create the {@link PIDController} object
     * @param kP the P coefficient
     * @param kI the I coefficient
     * @param kD the D coefficient
     * @param tolerance the controller's tolerance (what is "at point")
     */
    public MMPIDSubsystem(double kP, double kI, double kD, double tolerance) {
        pidController = new SQPIDController(kP, kI, kD);
        pidController.setTolerance(tolerance);
    }


    /**
     * default constructor to create the {@link PIDController} object
     * @param kP the P coefficient
     * @param kI the I coefficient
     * @param kD the D coefficient
     * @param tolerance the controller's tolerance (what is "at point")
     */
    public MMPIDSubsystem(double kP, double kI, double kD, double tolerance, double iZone) {
        pidController = new SQPIDController(kP, kI, kD);
        pidController.setTolerance(tolerance);
        pidController.setIZone(iZone);
    }

    /**
     * this constructor can be used in case u want to insert ur own pid controller
     * @param pidController ur pid controller object
     */
    public MMPIDSubsystem(SQPIDController pidController) {
        this.pidController = pidController;
    }

    public SQPIDController getPidController() {
        return pidController;
    }

    /**
     * this method should return the value to be fed into the PID controller
     * @return subsystem value
     */
    public abstract double getCurrentValue();

    /**
     * this method will help u add a kF component (optional).
     * @return the power to feed to the system
     */
    public double getFeedForwardPower() {
        return 0;
    }

    /**
     * this method will be called once the {@link MMPIDCommand#isFinished()} (atSetPoint)
     * <p>
     * this is also called when ur done tuning the FF power (using the {@link MMTuningFFCommand})
     * <p>
     * u can override it to make sure it stops ur subsystem.
     */
    public void stop() {
        setPower((double) 0);
    }

}
