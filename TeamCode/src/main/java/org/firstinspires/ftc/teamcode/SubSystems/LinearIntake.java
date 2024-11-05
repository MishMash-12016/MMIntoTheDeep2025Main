package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class LinearIntake extends SubsystemBase {
    private MMRobot robotInstance = MMRobot.getInstance();
    private final CuttleServo servoLeft;
    private final CuttleServo servoRight;

    public LinearIntake(){
        servoLeft=new CuttleServo(robotInstance.mmSystems.controlHub, Configuration.LEFT_SERVO_LINEAR_INTAKE_PORT);
        servoRight=new CuttleServo(robotInstance.mmSystems.controlHub, Configuration.LEFT_SERVO_LINEAR_INTAKE_PORT);
    }

    public Command SetPosition(double newPos){
        return new InstantCommand(()-> {
            servoLeft.setPosition(newPos);
        });
    }
}