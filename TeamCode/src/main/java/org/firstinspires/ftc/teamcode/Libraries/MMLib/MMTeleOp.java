package org.firstinspires.ftc.teamcode.Libraries.MMLib;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.AllianceSide;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


/**
 * this class represents a wrapper for the default Teleop.
 * <p>
 * if it is used while calling the side and/or color val it initializes the MMRobot with the {@link OpModeType.Competition#TELEOP Teleop} type.
 * u might also use the constructor that only requires a {@link OpModeType.NonCompetition  NonComp} type,
 * this constructor let's u insert the type of {@link OpModeType.NonCompetition  NonComp} opmode u would like to use.
 * there are explanations in {@link OpModeType} that explains the 3 options u have.
 * there is the {@link OpModeType.NonCompetition#DEBUG Debug},
 * {@link OpModeType.NonCompetition#EXPERIMENTING Experimenting},
 * {@link OpModeType.NonCompetition#EXPERIMENTING_NO_EXPANSION Experimenting Without Expansion}.
 *
 */
public abstract class MMTeleOp extends CommandOpMode {

    private final MMRobot mmRobot = MMRobot.getInstance();

    private OpModeType opModeType = OpModeType.Competition.TELEOP;

    private AllianceColor allianceColor;
    private AllianceSide allianceSide;

    private final List<Runnable> runOnInit = new ArrayList<>();
    private final List<Command> commandsOnRun = new ArrayList<>();


    /**
     * use this to choose a {@link OpModeType.NonCompetition NonComp} opmode.
     * @param opModeType which non-competition opmode to activate
     */
    public MMTeleOp(OpModeType.NonCompetition opModeType) {
        this.opModeType = opModeType;
    }

    /**
     * use this to initialize the competition teleop with the color only.
     * this is what u might want if the sides are irrelevant in ur teleop.
     * @param allianceColor the color of the alliance
     */
    public MMTeleOp(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    /**
     * this can be used if ur teleops are different according to ur alliance side.
     * @param allianceColor the color of the alliance
     * @param allianceSide the side of the alliance
     */
    public MMTeleOp(AllianceColor allianceColor, AllianceSide allianceSide) {
        this(allianceColor);
        this.allianceSide = allianceSide;
    }

    /**
     * this method runs on init.
     * <p>
     * first it initializes the robot instance,
     * then it runs the implementation of the {@link #onInit()} method the user specified,
     * and at last it schedules the commands and runnables.
     */
    @Override
    final public void initialize() {

        robotInit();

        onInit();

        scheduleCommandsAndRun();

    }

    /**
     * this initializes the {@link MMRobot} instance.
     */
    private void robotInit() {
        mmRobot.init(opModeType, allianceColor, allianceSide, hardwareMap, gamepad1, gamepad2, telemetry);
    }

    /**
     * this method helps you add {@link Runnable} or {@link Command} that will happen on init or run.
     * due to the fact that the {@link CommandScheduler CommandScheduler} run only when opmode is active,
     * you can't run commands on init, but you can run runnables on init. and commands right after the opmode is active.
     * use {@link #addRunnableOnInit(Runnable...)} and/or {@link #addCommandsOnRun(Command...)}.
     * <p>
     * leave it empty if you don't need them.
     * <p>
     * this ofc can also be used to add custom buttons (in experimenting) or whatever you might want.
     * <p>
     * this method runs on init.
     */
    public abstract void onInit();

    /**
     * this method helps u add methods that will run on the init stage. (before the actual match)
     * <p>
     * this can be used for setting the gripper position for example, or locking everything in place.
     * <p>
     * if you're using this, don't forget to add the sticker!
     * @param runOnInit methods to run
     */
    public void addRunnableOnInit(Runnable... runOnInit) {
        this.runOnInit.addAll(Arrays.asList(runOnInit));
    }

    /**
     * this method lets u add commands to run after the init stage.
     * <p>
     * (right after the match begins)
     * @param commandsOnRun commands to schedule
     */
    public void addCommandsOnRun(Command... commandsOnRun) {
        this.commandsOnRun.add(new InstantCommand().andThen(commandsOnRun));
        /*this was in order to solve the commandScheduler problem
          the problem was that the scheduler for some reason always ran the first instant command even tho it wasn't on yet*/
    }

    /**
     * this method runs the methods and schedules the commands that were specified in the {@link #onInit()} method.
     * <p>
     * (commands and methods that were added with the {@link #addRunnableOnInit(Runnable...)} and {@link  #addCommandsOnRun(Command...)})
     */
    private void scheduleCommandsAndRun() {
        for(Runnable runnable : runOnInit) {
            runnable.run();
        }

        for(Command command : commandsOnRun) {
            schedule(command);
        }
    }

    /**
     * this is ur loop in the opmode.
     * <p>
     * <b>IMPORTANT:</b>
     * <p>
     * the {@link CommandOpMode#run() super.run()} call, MUST be added in the start of the override block,
     * in order for the {@link CommandScheduler} to run.
     * <p>
     * the {@link CuttleRevHub#pullBulkData()} method needs to be called in order to update the (non-i2c) sensors.
     * (encoders - for example, wouldn't work otherwise)
     * <p>
     * the {@link Telemetry#update()} method is used to update the telemetry and send information to it.
     * <p>
     * use those in ur run method in the appropriate places.
     */
    @Override
    public void run() {
        super.run(); //MUST be added first.
        //override this method and add these lines as needed:
        /*
        mmRobot.mmSystems.controlHub.pullBulkData();    //updates the controlHub sensors
        mmRobot.mmSystems.expansionHub.pullBulkData();  //updates the expansionHub sensors
        telemetry.update();                             //updates the telemetry
        */
    }

    @Override
    public void reset() {
        super.reset();
        MMRobot.getInstance().resetRobot();
    }

}
