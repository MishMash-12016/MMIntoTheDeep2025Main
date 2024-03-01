package org.firstinspires.ftc.teamcode.Commands.utilCommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SelectCommand;

import org.firstinspires.ftc.teamcode.Commands.auto.Trajectories;
import org.firstinspires.ftc.teamcode.Commands.auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Utils.DetectionSide;
import org.firstinspires.ftc.teamcode.Utils.Side;

import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SideCommandSwitch extends SelectCommand {

    public SideCommandSwitch(CommandBase far, CommandBase center, CommandBase close, Supplier<Object> sideSupplier) {
        super(new HashMap<Object, Command>() {
                  {
                      put(DetectionSide.FAR, far);
                      put(DetectionSide.CENTER, center);
                      put(DetectionSide.CLOSE, close);
                  }
              }, sideSupplier
        );

    }

}