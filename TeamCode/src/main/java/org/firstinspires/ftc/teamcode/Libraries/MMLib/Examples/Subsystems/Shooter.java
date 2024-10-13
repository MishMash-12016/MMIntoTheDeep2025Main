package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Subsystems;

import com.roboctopi.cuttlefish.utils.Direction;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.SubsystemStructure.MMPowerPositionSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

/**
 * this class represents a subsystem which is both power and position based.
 * <p>
 * notice that in those cases if i try to use a command that controls only the setPosition,
 * and a command that controls only the setPower, they will both require the same subsystem
 * and be prevented from happening by the {@link com.arcrobotics.ftclib.command.CommandScheduler CommandScheduler},
 * while they are still physically able to move independently of one another, therefore, there is a logic problem here.
 * <p>
 * in this case, the solution is to separate the power and position, into 2 different subsystems.
 * <p>
 * see the CENTERSTAGE 2023-24 Intake.Lifter and Intake.Roller subsystems for example.
 */
public class Shooter extends MMPowerPositionSubsystem<Double, Double> {

    MMRobot mmRobot = MMRobot.getInstance();

    CuttleMotor motor1;
    CuttleMotor motor2;
    CuttleServo servo;

    public Shooter() {
        this.motor1 = mmRobot.mmSystems.controlHub.getMotor(Configuration.SHOOTER1);
        this.motor2 = mmRobot.mmSystems.controlHub.getMotor(Configuration.SHOOTER2);
        motor1.setDirection(Direction.REVERSE);
        this.servo = mmRobot.mmSystems.controlHub.getServo(Configuration.SHOOTER_SERVO);
    }

    @Override
    public void setPower(Double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    @Override
    public void setPosition(Double pos) {
        servo.setPosition(pos);
    }

    @Override
    public void periodic() {
        MMRobot.getInstance().mmSystems.telemetry.addData("Power", String.valueOf(motor1.getPower()));
    }
}
