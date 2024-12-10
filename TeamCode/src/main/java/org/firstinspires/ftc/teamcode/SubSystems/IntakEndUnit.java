package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.DoubleSupplier;

public class IntakEndUnit extends SubsystemBase {

    CuttleServo clawIntakeServo;
    public static final double open = 0.8;
    public static final double close = 0.2;

    // claw close or open
    public IntakEndUnit() {
        clawIntakeServo = new CuttleServo(MMRobot.getInstance().mmSystems.expansionHub, Configuration.CLAW_INTAKE_SERVO);
    }
    public Command openIntakeClaw(double open) {
        return new InstantCommand(() -> clawIntakeServo.setPosition(open), this);
    }

    public Command closeIntakeClaw(double close) {
        return new InstantCommand(() -> clawIntakeServo.setPosition(close), this);
    }

    public Command setPositionByJoystick(DoubleSupplier doubleSupplier){
        return new RunCommand(()-> {
            clawIntakeServo.setPosition(doubleSupplier.getAsDouble());
    },this);}

    public Double getPosition(){
        return clawIntakeServo.getPosition() ;
    }



}
