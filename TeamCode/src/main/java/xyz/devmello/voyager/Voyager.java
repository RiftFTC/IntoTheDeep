/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager;

import java.util.*;
import java.util.function.*;
import xyz.devmello.voyager.control.Controller;
import xyz.devmello.voyager.control.ProportionalController;
import xyz.devmello.voyager.exceptions.*;
import xyz.devmello.voyager.execution.ExecutorManager;
import xyz.devmello.voyager.follower.Follower;
import xyz.devmello.voyager.follower.FollowerGenerator;
import xyz.devmello.voyager.follower.generators.GenericFollowerGenerator;
import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.PointXY;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.geometry.Translation;
import xyz.devmello.voyager.listening.Listener;
import xyz.devmello.voyager.listening.ListenerBuilder;
import xyz.devmello.voyager.listening.ListenerManager;
import xyz.devmello.voyager.listening.ListenerMode;
import xyz.devmello.voyager.logging.Logger;
import xyz.devmello.voyager.math.Spline;
import xyz.devmello.voyager.math.Velocity;
import xyz.devmello.voyager.movement.MovementProfiler;
import xyz.devmello.voyager.plugin.PathfinderPlugin;
import xyz.devmello.voyager.plugin.PathfinderPluginManager;
import xyz.devmello.voyager.plugin.bundled.PositionLocker;
import xyz.devmello.voyager.plugin.bundled.StatTracker;
import xyz.devmello.voyager.prebuilt.AutoRotator;
import xyz.devmello.voyager.prebuilt.HeadingLock;
import xyz.devmello.voyager.recording.MovementPlayback;
import xyz.devmello.voyager.recording.MovementRecorder;
import xyz.devmello.voyager.recording.MovementRecording;
import xyz.devmello.voyager.recording.Recordable;
import xyz.devmello.voyager.recording.StateRecorder;
import xyz.devmello.voyager.robot.Drive;
import xyz.devmello.voyager.robot.Odometry;
import xyz.devmello.voyager.robot.Robot;
import xyz.devmello.voyager.robot.simulated.EmptyDrive;
import xyz.devmello.voyager.robot.simulated.EmptyOdometry;
import xyz.devmello.voyager.robot.simulated.SimulatedDrive;
import xyz.devmello.voyager.robot.simulated.SimulatedOdometry;
import xyz.devmello.voyager.robot.simulated.SimulatedRobot;
import xyz.devmello.voyager.scheduler.Scheduler;
import xyz.devmello.voyager.scheduler.Task;
import xyz.devmello.voyager.time.ElapsedTimer;
import xyz.devmello.voyager.time.Stopwatch;
import xyz.devmello.voyager.time.Time;
import xyz.devmello.voyager.trajectory.LinearTrajectory;
import xyz.devmello.voyager.trajectory.TaskTrajectory;
import xyz.devmello.voyager.trajectory.TaskTrajectoryBuilder;
import xyz.devmello.voyager.trajectory.Trajectory;
import xyz.devmello.voyager.trajectory.spline.AdvancedSplineTrajectoryBuilder;
import xyz.devmello.voyager.trajectory.spline.MultiSplineBuilder;
import xyz.devmello.voyager.utils.Button;
import xyz.devmello.voyager.utils.NotNull;
import xyz.devmello.voyager.utils.RandomString;
import xyz.devmello.voyager.utils.StringUtils;
import xyz.devmello.voyager.utils.ValidationUtils;
import xyz.devmello.voyager.zones.Zone;
import xyz.devmello.voyager.zones.ZoneProcessor;

/**
 * The highest-level interface used for interacting with {@code Pathfinder}.
 * This class is designed to give you complete (or near complete) control
 * over your robot's movement, whether autonomous or manually controlled.
 * There are a couple of key concepts you'll need to understand in order to
 * effectively make use of this class - namely, {@link Trajectory},
 * {@link Translation}, {@link PointXY}/{@link PointXYZ}, and {@link Angle}.
 * Additionally, the {@link #tick()} method is essential to operate Pathfinder.
 * I'd encourage you to go look at some documentation for the project to get
 * a decent idea of what's going on, but hey, that's up to you. Good luck...
 * I guess? Maybe? Yeah.
 *
 * <p>
 * This is absolutely a god class, but that's completely intentional, I swear.
 * Because Pathfinder fully supports method chaining, having a single object
 * ({@code Pathfinder}, in this case) with support for just about every
 * operation you could imagine makes method chaining incredibly easy and
 * effective. You can access smaller components of Pathfinder with some
 * getter methods:
 * <ul>
 *     <li>{@link #getDataMap()}</li>
 *     <li>{@link #getOdometry()}</li>
 *     <li>{@link #getDrive()}</li>
 *     <li>{@link #getExecutorManager()}</li>
 *     <li>{@link #getPluginManager()}</li>
 *     <li>{@link #getPlayback()}</li>
 *     <li>{@link #getMovementRecorder()}</li>
 *     <li>{@link #getListenerManager()}</li>
 * </ul>
 * Many of these can be disabled using minimal mode, which is explained in
 * the next paragraph.
 * </p>
 *
 * <p>
 * Pathfinder provides a global trajectory map that can be used to reference
 * trajectories via a {@link String} key, which can be convenient as it
 * allows trajectories to be accessed globally. To access this map, use
 * {@link #getTrajectoryMap()}. In addition to a
 * {@code Map<String, Trajectory>}, there's a {@code Map<String, String>}
 * that stores the stack trace of each of the trajectories added to the map,
 * so that in the event there's a naming conflict, it can be resolved
 * fairly quickly. To access that map, check out {@link #getStackTraceMap()}.
 * </p>
 *
 * <p>
 * Finally, there's a relatively minor feature called "minimal mode." It's
 * designed to decrease Pathfinder's footprint by skipping over the ticking
 * of (usually optional) services/managers/listeners. Using minimal mode
 * will restrict what Pathfinder is capable of, so I'd suggest you only make
 * use of it in situations where performance is absolutely critical. Because
 * unused services will usually have empty data structures, and it takes very
 * little time to iterate over an empty collection, ticking objects that
 * you're not using has a small impact on performance. To use minimal mode,
 * check out: {@link #setIsMinimal(boolean)}
 * </p>
 *
 * <p>
 * Using {@link StateRecorder}, Pathfinder can record the state of your robot,
 * allowing you to play it back later. This is different from movement
 * recording in that it records the state of several components of your robot
 * (ex: every motor) and plays back that exact state, removing any room for
 * error in playing back an existing path. To get started, first add some
 * {@link Recordable} nodes using {@link #addRecordable(String, Recordable)}.
 * Next, use {@link #getRecorder()} to access the {@link StateRecorder}.
 * From there, check out the documentation in the {@link StateRecorder} class
 * to learn how to record and play back the state of your robot.
 * </p>
 *
 * @author Colin Robertson
 * @author Pranav Yerramaneni
 * @see #goTo(PointXY)
 * @see #goTo(PointXYZ)
 * @see #setTranslation(Translation)
 * @see #followTrajectory(Trajectory)
 * @see #followTrajectories(List)
 * @see #follow(Follower)
 * @see #follow(List)
 * @see #tickUntil(double, Supplier)
 * @see #andThen(Consumer, double, Supplier)
 * @since 0.0.0
 */
@SuppressWarnings({ "UnusedReturnValue", "JavaDoc" })
public class Voyager {
    /**
     * A static list of plugins that should be automatically loaded every
     * time an instance of Pathfinder is created.
     */
    private static final List<PathfinderPlugin> AUTO_LOAD_PLUGINS = new ArrayList<>();

    /**
     * Globally-accessible map of trajectories.
     */
    private static final Map<String, Trajectory> TRAJECTORY_MAP = new HashMap<>();

    /**
     * Globally-accessible map of stack traces for where trajectories were
     * added from.
     */
    private static final Map<String, String> STACK_TRACE_MAP = new HashMap<>();

    /**
     * The {@code Robot} (made up of {@code Drive} and {@code Odometry}) that
     * Pathfinder operates.
     */
    private final Robot robot;

    /**
     * Pathfinder's executor manager.
     */
    private final ExecutorManager executorManager;

    /**
     * A generator used in converting trajectories into {@link Follower}s.
     */
    private final FollowerGenerator generator;

    /**
     * Turn controller, used for... controlling turns. What else would
     * it be used for, huh?
     */
    private final Controller turnController;

    /**
     * A stopwatch - this is very likely to not be useful at all, but its
     * included anyways.
     */
    private final Stopwatch stopwatch = new Stopwatch();

    /**
     * A zone processor is responsible for dealing with any zones on
     * the field.
     */
    private final ZoneProcessor zoneProcessor;

    /**
     * A scheduler for executing tasks.
     */
    private final Scheduler scheduler;

    /**
     * A manager for recording Pathfinder's movement.
     */
    private final MovementRecorder movementRecorder;

    /**
     * A manager for playing back recordings.
     */
    private final MovementPlayback playback;

    /**
     * A manager used for recording the state of several {@code Recordable}s.
     */
    private final StateRecorder recorder;

    /**
     * A manager for {@link PathfinderPlugin}s.
     */
    private final PathfinderPluginManager pluginManager;

    /**
     * Used in recording information about the robot's motion.
     */
    private final MovementProfiler profiler;

    /**
     * Used in event listeners.
     */
    private final ListenerManager listenerManager;

    /**
     * A modifiable map of operations to be run after every tick.
     */
    private final Map<String, Consumer<Voyager>> onTickOperations;

    /**
     * A map that can be used to communicate between classes.
     */
    private final Map<String, Object> dataMap;

    /**
     * The speed Pathfinder will use in creating linear trajectories.
     */
    private double speed = Core.pathfinderDefaultSpeed;

    /**
     * The tolerance Pathfinder will use in creating linear trajectories.
     */
    private double tolerance = Core.pathfinderDefaultTolerance;

    /**
     * The angle tolerance Pathfinder will use in creating linear trajectories.
     */
    private Angle angleTolerance = Core.pathfinderDefaultAngleTolerance;

    /**
     * Last tick, what was the currently active drive modifier? Or
     * something like that.
     */
    private Function<Translation, Translation> lastDriveModifier = null;

    /**
     * Is Pathfinder operating in minimal mode?
     */
    private boolean isMinimal = Core.pathfinderDefaultIsMinimal;

    /**
     * Create a new {@code Pathfinder} instance. This constructor will
     * conditionally load any automatically loading plugins - if the plugin's
     * name is not in the {@code doNotLoad} collection of strings, the
     * plugin will be loaded; if the plugin's name IS in the collection of
     * strings, the plugin will not be loaded.
     *
     * <p>
     * This constructor will instantiate instances of the following:
     * <ul>
     *     <li>{@link ExecutorManager}</li>
     *     <li>{@link ZoneProcessor}</li>
     *     <li>{@link Scheduler}</li>
     *     <li>{@link MovementRecorder}</li>
     *     <li>{@link MovementPlayback}</li>
     *     <li>{@link StateRecorder}</li>
     *     <li>{@link PathfinderPluginManager}</li>
     *     <li>{@link MovementProfiler}</li>
     *     <li>{@link ListenerManager}</li>
     * </ul>
     * There's a very good chance you're not going to need some or all of those,
     * and that's okay - you simply don't have to worry about them and everything
     * will work as intended. If that's the case, check out minimal mode
     * with {@link #setIsMinimal(boolean)}.
     * </p>
     *
     * <p>
     * Example implementation:
     * <code><pre>
     * public class Example {
     *     public void doStuff() {
     *         Drive drive = new SimulatedDrive();
     *         Odometry odometry = new SimulatedOdometry();
     *         Robot robot = new Robot(drive, odometry);
     *
     *         Controller turnController =
     *                 new ProportionalController(0.01);
     *         FollowGenerator generator =
     *                 new GenericFollowerGenerator(turnController);
     *
     *         Pathfinder pathfinder = new Pathfinder(
     *                 robot,
     *                 generator,
     *                 turnController,
     *                 "ExampleDoNotLoadPlugin1",
     *                 "ExampleDoNotLoadPlugin2
     *         );
     *     }
     * }
     * </pre></code>
     * Obviously, you shouldn't uses {@link SimulatedOdometry} or
     * {@link SimulatedDrive} for an actual robot, because then nothing
     * would work. But you get the point.
     * </p>
     *
     * @param robot          the {@code Pathfinder} instance's robot. This robot
     *                       should have an odometry system that can report the
     *                       position of the robot and a drive system that can
     *                       respond to drive commands. This object may not be
     *                       null or, an exception will be thrown.
     * @param generator      a generator used in creating followers. This generator
     *                       functions by accepting a {@link Trajectory} and a
     *                       {@link Robot} and returning a follower. If you're
     *                       unsure of what this means, or what you should do here,
     *                       you should probably use the "generic follower
     *                       generator," as it's the simplest. This object may not
     *                       be null, or an exception will be thrown.
     * @param turnController the controller responsible for turning the robot.
     *                       This is some bad code on my part, but basically,
     *                       this constructor assumes that the generator provided
     *                       makes use of a controller for controlling the
     *                       robot's heading.
     * @param doNotLoad      a set of {@code String}s that specify to Pathfinder
     *                       which plugins it should NOT load. If any of the
     *                       plugins that attempt to automatically load are inside
     *                       of the {@code doNotLoad} list, they won't be loaded.
     *                       This is here in case you depend on a file which
     *                       modifies the list of plugins that are automatically
     *                       loaded whenever an instance of Pathfinder is created.
     *                       If you decide you don't want that plugin to be loaded,
     *                       add it's name to this list, and you should be all good.
     */
    public Voyager(
        Robot robot,
        FollowerGenerator generator,
        Controller turnController,
        String... doNotLoad
    ) {
        if (robot == null) throw new NullPointerException(
            "Robot cannot be null!"
        );
        if (generator == null) throw new NullPointerException(
            "Follower generator cannot be null!"
        );

        Logger.debug(
            Voyager.class,
            "Created new Pathfinder instance (robot: <%s> generator: <%s> " +
            "turnController: <%s>)",
            robot,
            generator,
            turnController
        );

        this.robot = robot;
        this.generator = generator;
        this.turnController = turnController;
        this.executorManager = new ExecutorManager(robot);
        this.zoneProcessor = new ZoneProcessor();
        this.scheduler = new Scheduler(this);
        this.movementRecorder =
            new MovementRecorder(this, Core.movementRecorderMinDelayMs);
        this.playback = new MovementPlayback(this);
        this.recorder = new StateRecorder();
        this.pluginManager = new PathfinderPluginManager();
        this.profiler = new MovementProfiler();
        this.listenerManager = new ListenerManager(this);
        this.onTickOperations = new HashMap<>();
        this.dataMap = new HashMap<>();

        for (PathfinderPlugin plugin : AUTO_LOAD_PLUGINS) {
            String pluginName = plugin.getName();

            boolean shouldLoad = true;
            for (String str : doNotLoad) if (str.equals(pluginName)) {
                shouldLoad = false;
                break;
            }

            if (shouldLoad) loadPlugin(plugin);
        }
    }

    /**
     * Create a new {@code Pathfinder} instance.
     *
     * <p>
     * This constructor will instantiate instances of the following:
     * <ul>
     *     <li>{@link ExecutorManager}</li>
     *     <li>{@link ZoneProcessor}</li>
     *     <li>{@link Scheduler}</li>
     *     <li>{@link MovementRecorder}</li>
     *     <li>{@link MovementPlayback}</li>
     *     <li>{@link StateRecorder}</li>
     *     <li>{@link PathfinderPluginManager}</li>
     *     <li>{@link MovementProfiler}</li>
     *     <li>{@link ListenerManager}</li>
     * </ul>
     * There's a very good chance you're not going to need some or all of those,
     * and that's okay - you simply don't have to worry about them and everything
     * will work as intended. If that's the case, check out minimal mode
     * with {@link #setIsMinimal(boolean)}.
     * </p>
     *
     * <p>
     * Example implementation:
     * <code><pre>
     * public class Example {
     *     public void doStuff() {
     *         Drive drive = new SimulatedDrive();
     *         Odometry odometry = new SimulatedOdometry();
     *         Robot robot = new Robot(drive, odometry);
     *
     *         Controller turnController =
     *                 new ProportionalController(0.01);
     *         FollowGenerator generator =
     *                 new GenericFollowerGenerator(turnController);
     *
     *         Pathfinder pathfinder = new Pathfinder(
     *                 robot,
     *                 generator,
     *                 turnController
     *         );
     *     }
     * }
     * </pre></code>
     * Obviously, you shouldn't uses {@link SimulatedOdometry} or
     * {@link SimulatedDrive} for an actual robot, because then nothing
     * would work. But you get the point.
     * </p>
     */
    public Voyager(
        Robot robot,
        FollowerGenerator generator,
        Controller turnController
    ) {
        this(robot, generator, turnController, new String[0]);
    }

    /**
     * Create a new {@code Pathfinder} instance.
     *
     * @param robot     the {@code Pathfinder} instance's robot. This robot
     *                  should have an odometry system that can report the
     *                  position of the robot and a drive system that can
     *                  respond to drive commands. This object may not be
     *                  null or, an exception will be thrown.
     * @param generator a generator used in creating followers. This generator
     *                  functions by accepting a {@link Trajectory} and a
     *                  {@link Robot} and returning a follower. If you're
     *                  unsure of what this means, or what you should do here,
     *                  you should probably use the "generic follower
     *                  generator," as it's the simplest. This object may not
     *                  be null, or an exception will be thrown.
     */
    public Voyager(Robot robot, FollowerGenerator generator) {
        this(robot, generator, extractController(generator), new String[0]);
    }

    /**
     * Create a new {@code Pathfinder} instance.
     *
     * <p>
     * This constructor will instantiate instances of the following:
     * <ul>
     *     <li>{@link ExecutorManager}</li>
     *     <li>{@link ZoneProcessor}</li>
     *     <li>{@link Scheduler}</li>
     *     <li>{@link MovementRecorder}</li>
     *     <li>{@link MovementPlayback}</li>
     *     <li>{@link StateRecorder}</li>
     *     <li>{@link PathfinderPluginManager}</li>
     *     <li>{@link MovementProfiler}</li>
     *     <li>{@link ListenerManager}</li>
     * </ul>
     * There's a very good chance you're not going to need some or all of those,
     * and that's okay - you simply don't have to worry about them and everything
     * will work as intended. If that's the case, check out minimal mode
     * with {@link #setIsMinimal(boolean)}.
     * </p>
     *
     * <p>
     * Example implementation:
     * <code><pre>
     * public class Example {
     *     public void doStuff() {
     *         Drive drive = new SimulatedDrive();
     *         Odometry odometry = new SimulatedOdometry();
     *         Robot robot = new Robot(drive, odometry);
     *
     *         Controller turnController =
     *                 new ProportionalController(0.01);
     *
     *         Pathfinder pathfinder = new Pathfinder(
     *                 robot,
     *                 turnController
     *         );
     *     }
     * }
     * </pre></code>
     * Obviously, you shouldn't uses {@link SimulatedOdometry} or
     * {@link SimulatedDrive} for an actual robot, because then nothing
     * would work. But you get the point.
     * </p>
     *
     * @param robot          the {@code Pathfinder} instance's robot. This robot
     *                       should have an odometry system that can report the
     *                       position of the robot and a drive system that can
     *                       respond to drive commands. This object may not be
     *                       null or, an exception will be thrown.
     * @param turnController the controller responsible for turning the robot.
     *                       This is some bad code on my part, but basically,
     *                       this constructor assumes that the generator provided
     *                       makes use of a controller for controlling the
     *                       robot's heading.
     */
    public Voyager(Robot robot, Controller turnController) {
        this(
            robot,
            new GenericFollowerGenerator(turnController),
            turnController
        );
    }

    /**
     * Create a new {@code Pathfinder} instance.
     *
     * <p>
     * This constructor will instantiate instances of the following:
     * <ul>
     *     <li>{@link ExecutorManager}</li>
     *     <li>{@link ZoneProcessor}</li>
     *     <li>{@link Scheduler}</li>
     *     <li>{@link MovementRecorder}</li>
     *     <li>{@link MovementPlayback}</li>
     *     <li>{@link StateRecorder}</li>
     *     <li>{@link PathfinderPluginManager}</li>
     *     <li>{@link MovementProfiler}</li>
     *     <li>{@link ListenerManager}</li>
     * </ul>
     * There's a very good chance you're not going to need some or all of those,
     * and that's okay - you simply don't have to worry about them and everything
     * will work as intended. If that's the case, check out minimal mode
     * with {@link #setIsMinimal(boolean)}.
     * </p>
     *
     * <p>
     * Example implementation:
     * <code><pre>
     * public class Example {
     *     public void doStuff() {
     *         Drive drive = new SimulatedDrive();
     *         Odometry odometry = new SimulatedOdometry();
     *         Robot robot = new Robot(drive, odometry);
     *
     *         Pathfinder pathfinder = new Pathfinder(
     *                 robot,
     *                 0.01
     *         );
     *     }
     * }
     * </pre></code>
     * Obviously, you shouldn't uses {@link SimulatedOdometry} or
     * {@link SimulatedDrive} for an actual robot, because then nothing
     * would work. But you get the point.
     * </p>
     *
     * @param robot       the {@code Pathfinder} instance's robot. This robot
     *                    should have an odometry system that can report the
     *                    position of the robot and a drive system that can
     *                    respond to drive commands. This object may not be
     *                    null or, an exception will be thrown.
     * @param coefficient the coefficient used for the turn controller.
     */
    public Voyager(Robot robot, double coefficient) {
        this(robot, new ProportionalController(coefficient));
    }

    private static Controller extractController(FollowerGenerator generator) {
        if (generator instanceof GenericFollowerGenerator) {
            GenericFollowerGenerator gfg = (GenericFollowerGenerator) generator;
            return gfg.getTurnController();
        }

        throw new IllegalStateException(
            "Could not automatically extract " +
            "a turn controller from FollowerGenerator " +
            generator +
            ". Automatic turn controller extraction relies on using " +
            "GenericFollowerGenerator as a base class - if you " +
            "are NOT using that, you'll need to use the constructor " +
            "that allows you to specify a follower generator and " +
            "a turn controller."
        );
    }

    /**
     * Create a new, "simulated" instance of Pathfinder.
     *
     * <p>
     * This is pretty much only useful for debugging or testing purposes.
     * </p>
     *
     * @param coefficient the coefficient to use for the turn controller.
     * @return a new instance of Pathfinder, having a {@link Drive} of
     * {@link SimulatedDrive} and {@link Odometry} of {@link SimulatedOdometry}.
     */
    public static Voyager newSimulatedPathfinder(double coefficient) {
        Robot robot = new SimulatedRobot();

        return new Voyager(robot, coefficient)
            .setSpeed(0.5)
            .setTolerance(2)
            .setAngleTolerance(Angle.fromDeg(5));
    }

    /**
     * Create a new, "empty" instance of Pathfinder.
     *
     * <p>
     * This is pretty much only useful for debugging or testing purposes.
     * </p>
     *
     * @param coefficient the coefficient to use for the turn controller.
     * @return a new instance of Pathfinder that makes use of both the
     * {@link EmptyDrive} and {@link EmptyOdometry} classes.
     */
    public static Voyager newEmptyPathfinder(double coefficient) {
        Drive drive = new EmptyDrive();
        Odometry odometry = new EmptyOdometry();
        Robot robot = new Robot(drive, odometry);

        return new Voyager(robot, coefficient);
    }

    /**
     * Add a plugin to Pathfinder's list of automatically loading plugins.
     * These plugins will be loaded whenever an instance of Pathfinder
     * is created.
     *
     * @param plugin the plugin to load.
     */
    public static void addAutoLoadPlugin(PathfinderPlugin plugin) {
        AUTO_LOAD_PLUGINS.add(plugin);
    }

    private static String formatName(String group, String name) {
        return StringUtils.format("%s-%s", group, name);
    }

    /**
     * Add a {@code Trajectory} to the global trajectory map.
     *
     * @param group      the trajectory's group. A group is a group of
     *                   trajectories that's designed to improve organization.
     * @param name       the name of the trajectory.
     * @param trajectory the trajectory.
     */
    public static void addTrajectory(
        String group,
        String name,
        Trajectory trajectory
    ) {
        addTrajectory(formatName(group, name), trajectory);
    }

    /**
     * Add a trajectory to Pathfinder's global trajectory map.
     *
     * @param clazz      the class that added the trajectory to the map. This
     *                   is used for organizational purposes.
     * @param name       the name of the trajectory.
     * @param trajectory the trajectory to add.
     */
    public static void addTrajectory(
        Class<?> clazz,
        String name,
        Trajectory trajectory
    ) {
        addTrajectory(formatName(clazz.getSimpleName(), name), trajectory);
    }

    /**
     * Add a {@code Trajectory} to the global trajectory map.
     *
     * @param trajectoryName the name of the trajectory.
     * @param trajectory     the actual trajectory.
     */
    private static void addTrajectory(
        String trajectoryName,
        Trajectory trajectory
    ) {
        if (
            TRAJECTORY_MAP.containsKey(trajectoryName)
        ) throw new IllegalArgumentException(
            "Cannot add a trajectory " +
            "named <" +
            trajectoryName +
            "> because a trajectory " +
            "with that name already exists! The existing " +
            "trajectory was added from here: " +
            STACK_TRACE_MAP.get(trajectoryName)
        );

        StackTraceElement[] stackTrace = new Throwable().getStackTrace();

        if (stackTrace.length > 10) {
            StackTraceElement[] temp = new StackTraceElement[10];
            System.arraycopy(stackTrace, 0, temp, 0, 10);
            stackTrace = temp;
        }

        StringBuilder builder = new StringBuilder(stackTrace.length * 20);
        for (StackTraceElement element : stackTrace) {
            builder.append(element);
            builder.append("\n");
        }

        STACK_TRACE_MAP.put(trajectoryName, builder.toString());
        TRAJECTORY_MAP.put(trajectoryName, trajectory);
    }

    /**
     * Remove a {@code Trajectory} from the global trajectory map.
     *
     * @param trajectoryName the name of the trajectory.
     */
    public static void removeTrajectory(String trajectoryName) {
        STACK_TRACE_MAP.remove(trajectoryName);
        TRAJECTORY_MAP.remove(trajectoryName);
    }

    public static void removeTrajectory(String group, String name) {
        removeTrajectory(formatName(group, name));
    }

    /**
     * Remove a trajectory from Pathfinder's global trajectory map.
     *
     * @param clazz the class that added the trajectory.
     * @param name  the name of the trajectory.
     */
    public static void removeTrajectory(Class<?> clazz, String name) {
        removeTrajectory(formatName(clazz.getSimpleName(), name));
    }

    /**
     * Get a {@code Trajectory} from the global trajectory map.
     *
     * @param group the trajectory's group.
     * @param name  the trajectory's name.
     * @return the trajectory.
     */
    public static Trajectory getTrajectory(String group, String name) {
        return getTrajectory(formatName(group, name));
    }

    /**
     * Get a trajectory based on the trajectory's group/class and the
     * name of the trajectory. If the trajectory is not contained in the
     * trajectory map, this will throw a runtime exception.
     *
     * @param clazz the class that added the trajectory to the map. This is
     *              used for organizational purposes.
     * @param name  the trajectory's name.
     * @return the corresponding trajectory, if it's in the map.
     */
    public static Trajectory getTrajectory(Class<?> clazz, String name) {
        return getTrajectory(formatName(clazz.getSimpleName(), name));
    }

    /**
     * Get a {@code Trajectory} from the global trajectory map.
     *
     * @param trajectoryName the name of the trajectory.
     * @return the trajectory.
     */
    private static Trajectory getTrajectory(String trajectoryName) {
        if (
            !TRAJECTORY_MAP.containsKey(trajectoryName)
        ) throw new TrajectoryNotMappedException(
            "Cannot get a trajectory " +
            "with name <" +
            trajectoryName +
            "> because it has not " +
            "been added to the trajectory map! Use the " +
            "addTrajectory() method to do that."
        );

        return TRAJECTORY_MAP.get(trajectoryName);
    }

    /**
     * Clear Pathfinder's global trajectory map.
     */
    public static void clearTrajectoryMap() {
        STACK_TRACE_MAP.clear();
        TRAJECTORY_MAP.clear();
    }

    /**
     * Get Pathfinder's global trajectory map.
     *
     * @return Pathfinder's global trajectory map.
     */
    public static Map<String, Trajectory> getTrajectoryMap() {
        return TRAJECTORY_MAP;
    }

    /**
     * Get Pathfinder's global stack trace map. This map is used for debugging
     * purposes, to prevent users from adding a trajectory with the same
     * name twice. Ideally, you'll never even know that this map exists, but...
     * things don't always go ideally, unfortunately.
     *
     * @return Pathfinder's global stack trace map.
     */
    public static Map<String, String> getStackTraceMap() {
        return STACK_TRACE_MAP;
    }

    /**
     * Set Pathfinder's {@code isMinimal} status.
     *
     * @param isMinimal should Pathfinder run in minimal mode, which is
     *                  designed to reduce performance impact by skipping
     *                  un-needed operations?
     * @return {@code this}, used for method chaining.
     */
    public Voyager setIsMinimal(boolean isMinimal) {
        this.isMinimal = isMinimal;

        return this;
    }

    /**
     * Get Pathfinder's data map.
     *
     * @return Pathfinder's data map.
     */
    public Map<String, Object> getDataMap() {
        return dataMap;
    }

    /**
     * Add data to Pathfinder's data map.
     *
     * @param key    the key for the data.j
     * @param object the data to add.
     * @return {@code this}, used for method chaining.
     */
    public Voyager putData(String key, Object object) {
        dataMap.put(key, object);

        return this;
    }

    /**
     * Get data from Pathfinder's data map.
     *
     * @param key the key that corresponds with the data being accessed.
     * @return if the data map contains the requested key, return the
     * associated value. If the data map does not contain the requested key,
     * return null.
     */
    public Object getData(String key) {
        return dataMap.get(key);
    }

    /**
     * Get data from Pathfinder's data map and cast it to a specific
     * data type.
     *
     * @param key the key that corresponds with the data being accessed.
     * @return a casted result of a {@link #getData(String)} operation.
     */
    public PointXY getDataPointXY(String key) {
        return (PointXY) getData(key);
    }

    /**
     * Get data from Pathfinder's data map and cast it to a specific
     * data type.
     *
     * @param key the key that corresponds with the data being accessed.
     * @return a casted result of a {@link #getData(String)} operation.
     */
    public PointXYZ getDataPointXYZ(String key) {
        return (PointXYZ) getData(key);
    }

    /**
     * Get data from Pathfinder's data map and cast it to a specific
     * data type.
     *
     * @param key the key that corresponds with the data being accessed.
     * @return a casted result of a {@link #getData(String)} operation.
     */
    public Angle getDataAngle(String key) {
        return (Angle) getData(key);
    }

    /**
     * Get data from Pathfinder's data map and cast it to a specific
     * data type.
     *
     * @param key the key that corresponds with the data being accessed.
     * @return a casted result of a {@link #getData(String)} operation.
     */
    public String getDataString(String key) {
        return (String) getData(key);
    }

    /**
     * Load a {@link PathfinderPlugin}.
     *
     * @param plugin the plugin to load.
     * @return {@code this}, used for method chaining.
     */
    public Voyager loadPlugin(PathfinderPlugin plugin) {
        Logger.info(
            Voyager.class,
            "Loading plugin with name <%s>",
            plugin.getName()
        );

        pluginManager.loadPlugin(plugin);
        plugin.onLoad(this);

        return this;
    }

    /**
     * Load all of the plugins bundled with Pathfinder.
     * <p>
     * The plugins that will be loaded are (in order):
     * <ul>
     *     <li>{@link StatTracker}</li>
     *     <li>{@link PositionLocker}</li>
     * </ul>
     *
     * @return {@code this}, used for method chaining.
     */
    public Voyager loadBundledPlugins() {
        pluginManager.loadPlugin(new StatTracker());
        pluginManager.loadPlugin(new PositionLocker());

        return this;
    }

    /**
     * Get the {@code Pathfinder} instance's {@link Robot}. This is a final
     * field that's initialized upon construction.
     *
     * @return the {@code Pathfinder} instance's {@link Robot}.
     */
    public Robot getRobot() {
        return robot;
    }

    /**
     * Get the {@code Pathfinder} instance's {@link Odometry} system.
     *
     * @return the odometry system.
     */
    public Odometry getOdometry() {
        return robot.odometry();
    }

    /**
     * Get the {@code Pathfinder} instance's {@link Drive} system.
     *
     * @return the drive system.
     */
    public Drive getDrive() {
        return robot.drive();
    }

    /**
     * Get the {@code Pathfinder} instance's {@link MovementProfiler}.
     *
     * @return the {@link MovementProfiler}.
     */
    public MovementProfiler getProfiler() {
        return profiler;
    }

    /**
     * Get the {@code Pathfinder} instance's {@link ListenerManager}.
     *
     * @return the {@link ListenerManager}.
     */
    public ListenerManager getListenerManager() {
        return listenerManager;
    }

    /**
     * Add a listener to the listener manager.
     *
     * @param name     the name of the listener. This can usually just be
     *                 completely random, unless there's a need for it to
     *                 be something specific.
     * @param listener the listener that will be added.
     * @return {@code this}, used for method chaining.
     */
    public Voyager addListener(String name, Listener listener) {
        listenerManager.addListener(name, listener);

        return this;
    }

    /**
     * Add a listener to the listener manager.
     *
     * @param listener the listener that will be added.
     * @return {@code this}, used for method chaining.
     */
    public Voyager addListener(Listener listener) {
        return addListener(
            RandomString.randomString(Core.pathfinderRandomStringLength),
            listener
        );
    }

    /**
     * Add a listener.
     *
     * @param condition the condition that must be true in order for the
     *                  listener to be executed.
     * @param action    a piece of functionality to be executed whenever the
     *                  condition is met.
     * @return {@code this}, used for method chaining.
     */
    @SuppressWarnings("unchecked")
    public Voyager addListener(
        Predicate<Voyager> condition,
        Runnable action
    ) {
        Voyager voyager = this;

        return addListener(
            new Listener(
                ListenerMode.CONDITION_IS_MET,
                action,
                () -> condition.test(voyager)
            )
        );
    }

    /**
     * Get the speed at which Pathfinder will generate new linear followers.
     * This speed value is entirely irrelevant if you only generate custom
     * trajectories. It only applies to the {@link #goTo(PointXY)} and the
     * {@link #goTo(PointXYZ)} methods.
     *
     * @return the speed at which new linear followers will be generated.
     */
    public double getSpeed() {
        return this.speed;
    }

    /**
     * Set the speed Pathfinder will use to generate new linear trajectories.
     *
     * <p>
     * Note that this speed value only applies to two methods:
     * <ul>
     *     <li>{@link #goTo(PointXY)}</li>
     *     <li>{@link #goTo(PointXYZ)}</li>
     * </ul>
     * If you don't plan on using either of those methods, you can entirely
     * ignore the speed value.
     * </p>
     *
     * @param speed the speed at which Pathfinder should generate new linear
     *              trajectories when either of the two "goTo" methods
     *              are called.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager setSpeed(double speed) {
        InvalidSpeedException.throwIfInvalid(
            "Attempted to set speed to an invalid value - speed " +
            "values must be within the range of 0.0 to 1.0.",
            speed
        );

        this.speed = speed;

        return this;
    }

    /**
     * Get the tolerance Pathfinder will use for generating new linear
     * trajectories.
     *
     * @return the tolerance Pathfinder will use for generating new linear
     * trajectories.
     */
    public double getTolerance() {
        return this.tolerance;
    }

    /**
     * Set the tolerance Pathfinder will use for generating new linear
     * trajectories.
     *
     * <p>
     * This value only applies to two methods:
     * <ul>
     *     <li>{@link #goTo(PointXY)}</li>
     *     <li>{@link #goTo(PointXYZ)}</li>
     * </ul>
     * </p>
     *
     * @param tolerance the tolerance Pathfinder will use in generating
     *                  new linear trajectories.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager setTolerance(double tolerance) {
        InvalidToleranceException.throwIfInvalid(
            "Attempted to set an invalid tolerance - all " +
            "tolerance values must be above 0.",
            tolerance
        );

        this.tolerance = tolerance;

        return this;
    }

    /**
     * Get the angle tolerance Pathfinder will use in generating new linear
     * trajectories.
     *
     * @return the angle tolerance Pathfinder will use in generating new
     * linear trajectories.
     */
    public Angle getAngleTolerance() {
        return this.angleTolerance;
    }

    /**
     * Set the angle tolerance Pathfinder will use in generating new linear
     * trajectories.
     *
     * <p>
     * This value only applies to two methods:
     * <ul>
     *     <li>{@link #goTo(PointXY)}</li>
     *     <li>{@link #goTo(PointXYZ)}</li>
     * </ul>
     * </p>
     *
     * @param angleTolerance the tolerance Pathfinder should use in generating
     *                       new linear trajectories.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager setAngleTolerance(Angle angleTolerance) {
        InvalidToleranceException.throwIfInvalid(
            "Attempted to set an invalid angle tolerance - all " +
            "tolerance values must be above 0.",
            angleTolerance.deg()
        );

        this.angleTolerance = angleTolerance;

        return this;
    }

    /**
     * Get {@code this} instance of Pathfinder's {@link ZoneProcessor}.
     *
     * @return a {@link ZoneProcessor}.
     */
    public ZoneProcessor getZoneProcessor() {
        return zoneProcessor;
    }

    /**
     * Add a zone to collection of zones the processor is handling.
     *
     * @param name the name of the zone. This name will be needed if you're
     *             planning on removing the zone or referencing it at some
     *             point. This is also used for getting a list of names
     *             of zones.
     * @param zone the zone.
     * @return {@code this}, used for method chaining.
     */
    public Voyager addZone(String name, Zone zone) {
        zoneProcessor.addZone(name, zone);

        return this;
    }

    /**
     * Remove a zone.
     *
     * @param name the zone's name.
     * @return {@code this}, used for method chaining.
     */
    public Voyager removeZone(String name) {
        zoneProcessor.removeZone(name);

        return this;
    }

    /**
     * Get the ticks per second value. This is useful for measuring performance.
     * Requires the {@link StatTracker} plugin to be loaded.
     *
     * @return Pathfinder's ticks per second.
     */
    public double ticksPerSecond() {
        Object result = dataMap.get(StatTracker.KEY_TPS);

        if (result == null) throw new RuntimeException(
            "tried to get ticks per second without " +
            "having any valid entries - make sure you load the " +
            "StatTracker plugin!"
        ); else return (Double) result;
    }

    /**
     * Get Pathfinder's {@code Scheduler}.
     *
     * @return a {@link Scheduler}.
     */
    public Scheduler getScheduler() {
        return scheduler;
    }

    /**
     * Queue a single task.
     *
     * @param task the task the scheduler should execute.
     * @see Scheduler
     */
    public Voyager queueTask(Task task) {
        scheduler.queueTask(task);

        return this;
    }

    /**
     * Queue an array of tasks.
     *
     * @param tasks the array of tasks the scheduler should execute.
     * @see Scheduler
     */
    public Voyager queueTasks(Task... tasks) {
        scheduler.queueTasks(tasks);

        return this;
    }

    /**
     * Queue a list of tasks.
     *
     * @param tasks the list of tasks the scheduler should execute.
     * @see Scheduler
     */
    public Voyager queueTasks(List<Task> tasks) {
        scheduler.queueTasks(tasks);

        return this;
    }

    /**
     * Stop the {@code Scheduler} from doing anything at all. If the scheduler
     * is currently automatically ticking, this will stop it from doing
     * so.
     */
    public Voyager clearTasks() {
        scheduler.clear();

        return this;
    }

    /**
     * Get Pathfinder's movement recorder.
     *
     * <ul>
     *     <li>
     *         Start recording by getting Pathfinder's recorder and using the
     *         {@link MovementRecorder#start()} method. This will reset the current
     *         recording (so if you recorded something and then want to start over,
     *         this is how you would do that) and set the
     *         {@code isRecording} boolean to true. While this is true, whenever
     *         you call {@link Voyager#tick()}, the recorder will record
     *         information on Pathfinder's current movement.
     *     </li>
     *     <li>
     *         Once you've finished recording, use the {@link MovementRecorder#stop()}
     *         method to stop recording information. You can access this recorded
     *         data by using the {@link MovementRecorder#getRecording()} method.
     *     </li>
     *     <li>
     *         Now, to play back movement, it's pretty simple. You just use the
     *         {@link MovementPlayback#startPlayback(MovementRecording)} method
     *         to start the playback, and then use {@link Voyager#tick()} to
     *         continue playing the movement back.
     *     </li>
     *     <li>
     *         Because everything is done on a single thread, it's quite easy
     *         to stop or start recording, and you won't have any issues with
     *         doing just that.
     *     </li>
     * </ul>
     *
     * @return Pathfinder's movement recorder.
     */
    public MovementRecorder getMovementRecorder() {
        return movementRecorder;
    }

    /**
     * Get Pathfinder's movement playback manager.
     *
     * <ul>
     *     <li>
     *         Start recording by getting Pathfinder's recorder and using the
     *         {@link MovementRecorder#start()} method. This will reset the current
     *         recording (so if you recorded something and then want to start over,
     *         this is how you would do that) and set the
     *         {@code isRecording} boolean to true. While this is true, whenever
     *         you call {@link Voyager#tick()}, the recorder will record
     *         information on Pathfinder's current movement.
     *     </li>
     *     <li>
     *         Once you've finished recording, use the {@link MovementRecorder#stop()}
     *         method to stop recording information. You can access this recorded
     *         data by using the {@link MovementRecorder#getRecording()} method.
     *     </li>
     *     <li>
     *         Now, to play back movement, it's pretty simple. You just use the
     *         {@link MovementPlayback#startPlayback(MovementRecording)} method
     *         to start the playback, and then use {@link Voyager#tick()} to
     *         continue playing the movement back.
     *     </li>
     *     <li>
     *         Because everything is done on a single thread, it's quite easy
     *         to stop or start recording, and you won't have any issues with
     *         doing just that.
     *     </li>
     * </ul>
     *
     * @return Pathfinder's movement playback manager.
     */
    public MovementPlayback getPlayback() {
        return playback;
    }

    /**
     * Get Pathfinder's {@code StateRecorder} instance. In order to record,
     * you first need to add recordable nodes. To add a node to this
     * {@code StateRecorder}, use {@link #addRecordable(String, Recordable)}.
     *
     * @return Pathfinder's {@code StateRecorder}.
     */
    public StateRecorder getRecorder() {
        return recorder;
    }

    /**
     * Add a {@code Recordable} to Pathfinder's {@code StateRecorder}.
     *
     * @param name       the name of the node. This should be the same name
     *                   used in any recordings you plan on playing back.
     * @param recordable the {@code Recordable} to add.
     * @return {@code this}, used for method chaining.
     */
    public Voyager addRecordable(String name, Recordable<?> recordable) {
        recorder.putNode(name, recordable);

        return this;
    }

    /**
     * Get Pathfinder's plugin manager. This manager can be used to load
     * and unload plugins, allowing you to customize Pathfinder's inner
     * workings to your likings.
     *
     * @return Pathfinder's plugin manager.
     */
    public PathfinderPluginManager getPluginManager() {
        return pluginManager;
    }

    private void runOnTickOperations() {
        for (Map.Entry<String, Consumer<Voyager>> entry : onTickOperations.entrySet()) {
            Consumer<Voyager> consumer = entry.getValue();

            consumer.accept(this);
        }
    }

    /**
     * Bind an operation to the invocation of Pathfinder's {@link #tick()}
     * method. Anything that's bound to Pathfinder's {@link #tick()} method
     * will be called at the end of the invocation of {@link #tick()}.
     *
     * @param name   the name of the on tick operation. This mostly exists
     *               just so you can later remove on tick operations with
     *               {@link #removeOnTick(String)}.
     * @param onTick an action to be executed whenever Pathfinder ticks. This
     *               will be executed right before the plugin post-tick stuff,
     *               meaning it's after everything else. This {@link Consumer}
     *               accepts an instance of {@link Voyager}, which will
     *               always be the instance that was just ticked.
     * @return {@code this}, used for method chaining.
     */
    public Voyager onTick(String name, Consumer<Voyager> onTick) {
        onTickOperations.put(name, onTick);

        return this;
    }

    /**
     * Bind an operation to the invocation of Pathfinder's {@link #tick()}
     * method. Anything that's bound to Pathfinder's {@link #tick()} method
     * will be called at the end of the invocation of {@link #tick()}.
     *
     * @param onTick an action to be executed whenever Pathfinder ticks. This
     *               will be executed right before the plugin post-tick stuff,
     *               meaning it's after everything else. This {@link Consumer}
     *               accepts an instance of {@link Voyager}, which will
     *               always be the instance that was just ticked.
     * @return {@code this}, used for method chaining.
     */
    public Voyager onTick(Consumer<Voyager> onTick) {
        return onTick(
            RandomString.randomString(Core.pathfinderRandomStringLength),
            onTick
        );
    }

    /**
     * Bind an operation to the invocation of Pathfinder's {@link #tick()}
     * method. Anything that's bound to Pathfinder's {@link #tick()} method
     * will be called at the end of the invocation of {@link #tick()}.
     *
     * @param onTick an action to be executed whenever Pathfinder ticks. This
     *               will be executed right before the plugin post-tick stuff,
     *               meaning it's after everything else.
     * @return {@code this}, used for method chaining.
     */
    public Voyager onTick(Runnable onTick) {
        return onTick(pf -> onTick.run());
    }

    /**
     * Bind an operation to the invocation of Pathfinder's {@link #tick()}
     * method. This utilizes Pathfinder's {@link ListenerManager} to accommodate
     * for more advanced features, such as expiration time, cooldown, and
     * the maximum number of executions.
     *
     * @param onTick         an action to be executed whenever Pathfinder
     *                       ticks. This will be executed right before the
     *                       plugin post-tick stuff, meaning it's after
     *                       everything else.
     * @param minimumDelayMs the minimum delay between executions of the
     *                       {@code Runnable}, in milliseconds.
     * @param expiration     the time the listener will expire, represented
     *                       in milliseconds since Jan 1st, 1970.
     * @param maxExecs       the maximum amount of times the listener can
     *                       be executed before automatically being dequeued.
     * @param priority       the event's priority. Events with a higher
     *                       priority will be executed before events with
     *                       a lower priority. Defaults to 0.
     * @return {@code this}, used for method chaining.
     */
    public Voyager onTick(
        Runnable onTick,
        double minimumDelayMs,
        double expiration,
        int maxExecs,
        int priority
    ) {
        return addListener(
            new ListenerBuilder()
                .setPriority(0)
                .setMode(ListenerMode.CONDITION_IS_MET)
                .addInput(() -> true)
                .setExpiration(expiration)
                .setCooldownMs(minimumDelayMs)
                .setWhenTriggered(onTick)
                .setMaximumExecutions(maxExecs)
                .build()
        );
    }

    /**
     * Bind an operation to the invocation of Pathfinder's {@link #tick()}
     * method. This utilizes Pathfinder's {@link ListenerManager} to accommodate
     * for more advanced features, such as expiration time, cooldown, and
     * the maximum number of executions.
     *
     * @param onTick         an action to be executed whenever Pathfinder
     *                       ticks. This will be executed right before the
     *                       plugin post-tick stuff, meaning it's after
     *                       everything else.
     * @param minimumDelayMs the minimum delay between executions of the
     *                       {@code Runnable}, in milliseconds.
     * @return {@code this}, used for method chaining.
     */
    public Voyager onTick(Runnable onTick, double minimumDelayMs) {
        return onTick(
            onTick,
            minimumDelayMs,
            Core.listenerBuilderDefaultExpiration,
            Core.listenerBuilderDefaultMaximumExecutions,
            Core.listenerBuilderDefaultPriority
        );
    }

    /**
     * Remove an on tick operation.
     *
     * @param name the name of the operation to remove.
     * @return {@code this}, used for method chaining.
     */
    public Voyager removeOnTick(String name) {
        onTickOperations.remove(name);

        return this;
    }

    private Voyager runPreTick() {
        return TickProcessor.runPreTick(
            this,
            isMinimal,
            pluginManager,
            scheduler,
            zoneProcessor
        );
    }

    private Voyager runExecutorTick() {
        return TickProcessor.runExecutorTick(this, executorManager);
    }

    private Voyager runOnTick() {
        return TickProcessor.runOnTick(
            this,
            isMinimal,
            pluginManager,
            playback,
            profiler,
            movementRecorder,
            recorder,
            listenerManager,
            this::runOnTickOperations
        );
    }

    private static Voyager runPostTick(
        Voyager voyager,
        PathfinderPluginManager plugins
    ) {
        plugins.postTick(voyager);

        return voyager;
    }

    private Voyager runPostTick() {
        return runPostTick(this, pluginManager);
    }

    /**
     * "Tick" Pathfinder once. This will tell Pathfinder's execution manager
     * to check to see what Pathfinder should be doing right now, and based
     * on that, move your robot. This method is required to operate Pathfinder
     * and should be run as frequently as possible. Not executing this method
     * will cause the library to not function at all.
     *
     * <p>
     * If the {@code tick()} method causes the invocation of an odometry
     * system's {@link Odometry#getPosition()} method, and that method updates
     * the robot's position based on the amount of elapsed time since the
     * last update, there can be issues. If {@code tick()} is called too
     * frequently, there may be inaccuracy in positional tracking if the
     * {@link Odometry#getPosition()} method is called too frequently.
     * </p>
     *
     * <p>
     * The order everything is ticked in is as follows:
     * <ol>
     *     <li>Plugin pre-tick ({@link PathfinderPluginManager#preTick(Voyager)})</li>
     *     <li>Scheduler ({@link #getScheduler()})</li>
     *     <li>Zone processor ({@link #getZoneProcessor()})</li>
     *     <li>Executor manager ({@link #getExecutorManager()})</li>
     *     <li>Playback manager ({@link #getPlayback()})</li>
     *     <li>Motion profiler ({@link #getProfiler()})</li>
     *     <li>Recording manager ({@link #getMovementRecorder()})</li>
     *     <li>On tick operations ({@link #onTickOperations})</li>
     *     <li>Plugin post-tick ({@link PathfinderPluginManager#postTick(Voyager)})</li>
     * </ol>
     * </p>
     *
     * <p>
     * If you're curious about how many times the tick method is being called
     * per second, check out {@link #ticksPerSecond()}. This does require you
     * to use the {@link StatTracker} plugin, which can be loaded two ways:
     * <ul>
     *     <li>{@code pathfinder.loadPlugin(new StatTracker());}</li>
     *     <li>{@code pathfinder.loadBundledPlugins();}</li>
     * </ul>
     * </p>
     *
     * @return {@code this}, used for method chaining.
     */
    public Voyager tick() {
        return runPreTick().runExecutorTick().runOnTick().runPostTick();
    }

    /**
     * Tick Pathfinder for a certain amount of time, specified in milliseconds.
     * This will block the current thread until the provided amount of time
     * (in milliseconds) has elapsed. The {@link #tick()} method will be
     * called as frequently as possible while the amount of elapsed time
     * is less than the provided time.
     *
     * <p>
     * This method makes use of {@link ElapsedTimer} to measure the elapsed
     * time. More specifically: {@link ElapsedTimer#runFor(Runnable, double)}
     * </p>
     *
     * @param timeMs the amount of time, in milliseconds, that Pathfinder
     *               should be continually ticked for.
     * @return {@code this}, used for method chaining.
     * @see #tick()
     * @see #tickUntil()
     * @see #tickUntil(double)
     */
    public Voyager tickFor(double timeMs) {
        ValidationUtils.validate(timeMs, "timeMs");

        ElapsedTimer.runFor(this::tick, timeMs);

        return this;
    }

    /**
     * Tick Pathfinder according to a {@code TickConfig}.
     *
     * @param config the ticking configuration to use.
     * @return {@code this}, used for method chaining.
     */
    public Voyager tickUntil(TickConfig config) {
        double delayMs = config.getDelayMs();

        if (delayMs > 0) {
            ElapsedTimer delayTimer = new ElapsedTimer(true);

            while (delayTimer.elapsedMs() <= delayMs);
        }

        double timeoutMs = config.getTimeoutMs();
        ElapsedTimer timer = new ElapsedTimer(true);

        List<Supplier<Boolean>> shouldContinueRunning = config.getShouldContinueRunning();
        List<Supplier<Boolean>> shouldStopRunning = config.getShouldStopRunning();
        List<Runnable> onTick = config.getOnTick();
        List<Runnable> onFinish = config.getOnFinish();

        while (timer.elapsedMs() <= timeoutMs) {
            for (Supplier<Boolean> supplier : shouldContinueRunning) {
                if (!supplier.get()) {
                    break;
                }
            }

            for (Supplier<Boolean> supplier : shouldStopRunning) {
                if (supplier.get()) {
                    break;
                }
            }

            tick();

            for (Runnable runnable : onTick) {
                runnable.run();
            }
        }

        for (Runnable runnable : onFinish) {
            runnable.run();
        }

        return this;
    }

    /**
     * Tick Pathfinder until it finishes whatever path is currently being
     * executed.
     *
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager tickUntil() {
        return tickUntil(Double.MAX_VALUE, () -> true);
    }

    /**
     * Tick Pathfinder until either the path it was following is finished or
     * the timeout time (in milliseconds) is reached.
     *
     * @param timeoutMs how long, in milliseconds, Pathfinder will
     *                  continue ticking (as a maximum). If the
     *                  path finishes before this time is reached,
     *                  it'll stop as normal.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager tickUntil(double timeoutMs) {
        return tickUntil(timeoutMs, () -> true);
    }

    /**
     * Tick Pathfinder while the provided supplier returns true.
     *
     * @param shouldContinueRunning a supplier, indicating whether Pathfinder
     *                              should still continue running.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager tickUntil(Supplier<Boolean> shouldContinueRunning) {
        return tickUntil(Double.MAX_VALUE, shouldContinueRunning);
    }

    /**
     * Tick Pathfinder while the provided supplier returns true and the
     * elapsed time is less than the timeout time.
     *
     * @param timeoutMs             how long, in milliseconds, Pathfinder will
     *                              continue ticking (as a maximum). If the
     *                              path finishes before this time is reached,
     *                              it'll stop as normal.
     * @param shouldContinueRunning a supplier, indicating whether Pathfinder
     *                              should still continue running.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager tickUntil(
        double timeoutMs,
        Supplier<Boolean> shouldContinueRunning
    ) {
        return tickUntil(timeoutMs, shouldContinueRunning, pathfinder -> {});
    }

    /**
     * Tick Pathfinder while the provided supplier returns true.
     *
     * @param shouldContinueRunning a supplier, indicating whether Pathfinder
     *                              should still continue running.
     * @param onTick                a {@link Consumer} that will be executed
     *                              after every successful tick.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager tickUntil(
        Supplier<Boolean> shouldContinueRunning,
        Consumer<Voyager> onTick
    ) {
        return tickUntil(Double.MAX_VALUE, shouldContinueRunning, onTick);
    }

    /**
     * Tick Pathfinder while the provided supplier returns true and the
     * elapsed time is less than the timeout time.
     *
     * @param timeoutMs             how long, in milliseconds, Pathfinder will
     *                              continue ticking (as a maximum). If the
     *                              path finishes before this time is reached,
     *                              it'll stop as normal.
     * @param shouldContinueRunning a supplier, indicating whether Pathfinder
     *                              should still continue running.
     * @param onTick                a {@link Consumer} that will be executed
     *                              after every successful tick.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager tickUntil(
        double timeoutMs,
        Supplier<Boolean> shouldContinueRunning,
        Consumer<Voyager> onTick
    ) {
        if (timeoutMs < 0) throw new InvalidTimeException(
            "Attempted to use an invalid timeout time in in a call to " +
            "the tickUntil method - make sure this time value " +
            "is greater than or equal to 0."
        );

        if (shouldContinueRunning == null) throw new NullPointerException(
            "Attempted to use a null supplier with the tickUntil " +
            "method - this can't be null, nerd."
        );

        if (onTick == null) throw new NullPointerException(
            "Attempted to use a null consumer with the tickUntil " +
            "method. This also can't be null, nerd."
        );

        double start = Time.ms();

        while (isActive() && shouldContinueRunning.get()) {
            double current = Time.ms();
            double elapsed = current - start;

            if (elapsed >= timeoutMs) break;

            tick();
            onTick.accept(this);
        }

        return this;
    }

    /**
     * Tick Pathfinder while the provided supplier returns true and the
     * elapsed time is less than the timeout time.
     *
     * @param shouldContinueRunning a supplier, indicating whether Pathfinder
     *                              should still continue running.
     * @param timeoutMs             how long, in milliseconds, Pathfinder will
     *                              continue ticking (as a maximum). If the
     *                              path finishes before this time is reached,
     *                              it'll stop as normal.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager tickUntil(
        Supplier<Boolean> shouldContinueRunning,
        double timeoutMs
    ) {
        return tickUntil(timeoutMs, shouldContinueRunning, (a, b) -> {});
    }

    /**
     * Tick Pathfinder while the provided supplier returns true and the
     * elapsed time is less than the timeout time.
     *
     * @param timeoutMs             how long, in milliseconds, Pathfinder will
     *                              continue ticking (as a maximum). If the
     *                              path finishes before this time is reached,
     *                              it'll stop as normal.
     * @param shouldContinueRunning a supplier, indicating whether Pathfinder
     *                              should still continue running.
     * @param onTick                a {@link Consumer} that will be executed
     *                              after every successful tick. This consumer
     *                              accepts two parameters - first the instance
     *                              of Pathfinder that is running. Second, a
     *                              double value representing the total elapsed
     *                              time (in milliseconds) that the tick
     *                              until method has been running for.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager tickUntil(
        double timeoutMs,
        Supplier<Boolean> shouldContinueRunning,
        BiConsumer<Voyager, Double> onTick
    ) {
        NotNull.throwExceptionIfNull(
            "A null value was passed to the tickUntil method! " +
            "Please make sure you don't pass any null values.",
            shouldContinueRunning,
            onTick
        );

        double start = Time.ms();

        while (isActive() && shouldContinueRunning.get()) {
            double current = Time.ms();
            double elapsed = current - start;

            if (elapsed >= timeoutMs) break;

            tick();
            onTick.accept(this, elapsed);
        }

        return this;
    }

    /**
     * Continually tick Pathfinder for as long as it needs to be ticked to
     * finish executing the current path. This method accepts a
     * {@link BiConsumer} parameter that in turn accepts two parameters -
     * first, {@code this} instance of Pathfinder, and second, the elapsed
     * time (in milliseconds). If you'd like to exit out of the ticking, you
     * can simply use {@link #clear()} inside of the {@link BiConsumer}.
     *
     * @param onTick a {@link Consumer} that will be executed
     *               after every successful tick. This consumer
     *               accepts two parameters - first the instance
     *               of Pathfinder that is running. Second, a
     *               double value representing the total elapsed
     *               time (in milliseconds) that the tick
     *               until method has been running for.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager tickUntil(BiConsumer<Voyager, Double> onTick) {
        double start = Time.ms();

        while (isActive()) {
            double current = Time.ms();
            double elapsed = current - start;

            onTick.accept(this, elapsed);
        }

        return this;
    }

    /**
     * Tick Pathfinder while the elapsed time is less than the timeout (in
     * milliseconds) and the {@link Predicate} (accepting {@code this} as
     * a parameter) returns true.
     *
     * @param timeoutMs how long, in milliseconds, Pathfinder will
     *                  continue ticking (as a maximum). If the
     *                  path finishes before this time is reached,
     *                  it'll stop as normal.
     * @param isValid   a predicate, accepting {@code this}
     *                  instance of Pathfinder as a parameter.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager tickUntil(
        double timeoutMs,
        Predicate<Voyager> isValid
    ) {
        NotNull.throwExceptionIfNull(
            "A null value was passed to the tickUntil method! " +
            "Please make sure you don't pass any null values.",
            isValid
        );

        double start = Time.ms();

        while (isActive() && isValid.test(this)) {
            double current = Time.ms();
            double elapsed = current - start;

            if (elapsed >= timeoutMs) break;

            tick();
        }

        return this;
    }

    /**
     * Tick Pathfinder while the elapsed time is less than the timeout (in
     * milliseconds), the {@link Predicate} (accepting {@code this} as
     * a parameter) returns true, and the provided {@link Supplier} also
     * returns true.
     *
     * @param timeoutMs             how long, in milliseconds, Pathfinder will
     *                              continue ticking (as a maximum). If the
     *                              path finishes before this time is reached,
     *                              it'll stop as normal.
     * @param shouldContinueRunning a supplier, indicating whether Pathfinder
     *                              should still continue running.
     * @param isValid               a predicate, accepting {@code this}
     *                              instance of Pathfinder as a parameter.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager tickUntil(
        double timeoutMs,
        Supplier<Boolean> shouldContinueRunning,
        Predicate<Voyager> isValid
    ) {
        NotNull.throwExceptionIfNull(
            "A null value was passed to the tickUntil method! " +
            "Please make sure you don't pass any null values.",
            shouldContinueRunning,
            isValid
        );

        double start = Time.ms();

        while (
            isActive() && shouldContinueRunning.get() && isValid.test(this)
        ) {
            double current = Time.ms();
            double elapsed = current - start;

            if (elapsed >= timeoutMs) break;

            tick();
        }

        return this;
    }

    /**
     * Use the {@code tickUntil} method to tick Pathfinder until the path
     * it's executing is finished.
     *
     * @param onCompletion a callback to be executed after Pathfinder finishes
     *                     whatever it's doing. This consumer accepts the
     *                     instance of Pathfinder that this method was
     *                     called from.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager andThen(Consumer<Voyager> onCompletion) {
        return andThen(onCompletion, Double.MAX_VALUE, () -> true);
    }

    /**
     * Use the {@code tickUntil} method to tick Pathfinder until the path
     * it's executing is finished (or a timeout is reached).
     *
     * @param onCompletion a callback to be executed after Pathfinder finishes
     *                     whatever it's doing. This consumer accepts the
     *                     instance of Pathfinder that this method was
     *                     called from.
     * @param timeoutMs    how long, in milliseconds, Pathfinder will continue
     *                     ticking (as a maximum). If the path finishes before
     *                     this time is reached, it'll stop as normal.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager andThen(
        Consumer<Voyager> onCompletion,
        double timeoutMs
    ) {
        return andThen(onCompletion, timeoutMs, () -> true);
    }

    /**
     * Use the {@code tickUntil} method to tick Pathfinder until the path
     * it's executing is finished (or the {@code shouldContinueRunning}
     * {@link Supplier} returns false).
     *
     * @param onCompletion          a callback to be executed after Pathfinder finishes
     *                              whatever it's doing. This consumer accepts the
     *                              instance of Pathfinder that this method was
     *                              called from.
     * @param shouldContinueRunning a supplier, indicating whether Pathfinder
     *                              should still continue running.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager andThen(
        Consumer<Voyager> onCompletion,
        Supplier<Boolean> shouldContinueRunning
    ) {
        return andThen(onCompletion, Double.MAX_VALUE, shouldContinueRunning);
    }

    /**
     * Use the {@code tickUntil} method to tick Pathfinder until the path
     * it's executing is finished (or a timeout is reached, or the
     * {@code shouldContinueRunning} {@link Supplier} returns false).
     *
     * @param onCompletion          a callback to be executed after Pathfinder finishes
     *                              whatever it's doing. This consumer accepts the
     *                              instance of Pathfinder that this method was
     *                              called from.
     * @param timeoutMs             how long, in milliseconds, Pathfinder will continue
     *                              ticking (as a maximum). If the path finishes before
     *                              this time is reached, it'll stop as normal.
     * @param shouldContinueRunning a supplier, indicating whether Pathfinder
     *                              should still continue running.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager andThen(
        Consumer<Voyager> onCompletion,
        double timeoutMs,
        Supplier<Boolean> shouldContinueRunning
    ) {
        tickUntil(timeoutMs, shouldContinueRunning);

        onCompletion.accept(this);

        return this;
    }

    /**
     * Use the {@code tickUntil} method to tick Pathfinder until the path
     * it's executing is finished (or a timeout is reached, or the
     * {@code shouldContinueRunning} {@link Supplier} returns false).
     *
     * @param onCompletion          a callback to be executed after Pathfinder finishes
     *                              whatever it's doing. This consumer accepts the
     *                              instance of Pathfinder that this method was
     *                              called from.
     * @param timeoutMs             how long, in milliseconds, Pathfinder will continue
     *                              ticking (as a maximum). If the path finishes before
     *                              this time is reached, it'll stop as normal.
     * @param shouldContinueRunning a supplier, indicating whether Pathfinder
     *                              should still continue running.
     * @param onTick                a {@link Consumer} that will be called
     *                              once per tick.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager andThen(
        Consumer<Voyager> onCompletion,
        double timeoutMs,
        Supplier<Boolean> shouldContinueRunning,
        BiConsumer<Voyager, Double> onTick
    ) {
        tickUntil(timeoutMs, shouldContinueRunning, onTick);

        onCompletion.accept(this);

        return this;
    }

    /**
     * Execute a {@link Runnable} action until either (1) the elapsed time, in
     * milliseconds, exceeds {@code maximumTimeMs} or (2) the {@code condition}
     * supplier returns false.
     *
     * @param maximumTimeMs the maximum time, in milliseconds, this method
     *                      will be allowed to execute for.
     * @param condition     the condition that must be true in order for
     *                      this method to continue execution.
     * @param action        the action that will be run repeatedly.
     * @return {@code this}, used for method chaining.
     */
    public Voyager asLongAs(
        double maximumTimeMs,
        Supplier<Boolean> condition,
        Runnable action
    ) {
        ElapsedTimer timer = new ElapsedTimer(true);

        while (condition.get() && timer.isElapsedLessThan(maximumTimeMs)) {
            action.run();
        }

        return this;
    }

    /**
     * Execute a {@link Runnable} action until either (1) the elapsed time, in
     * milliseconds, exceeds {@code maximumTimeMs} or (2) the {@code predicate}
     * {@link Predicate} returns false.
     *
     * @param maximumTimeMs the maximum time, in milliseconds, this method
     *                      will be allowed to execute for.
     * @param predicate     a {@link Predicate} that must return true in order
     *                      order for the method to continue executing. If the
     *                      predicate returns false, the method's execution
     *                      will terminate.
     * @param action        the action that will be run repeatedly.
     * @return {@code this}, used for method chaining.
     */
    public Voyager asLongAs(
        double maximumTimeMs,
        Predicate<Voyager> predicate,
        Runnable action
    ) {
        return asLongAs(maximumTimeMs, () -> predicate.test(this), action);
    }

    /**
     * Execute a {@link Runnable} action until the {@code condition} supplier
     * returns false.
     *
     * @param condition the condition that must be true in order for
     *                  this method to continue execution.
     * @param action    the action that will be run repeatedly.
     * @return {@code this}, used for method chaining.
     */
    public Voyager asLongAs(Supplier<Boolean> condition, Runnable action) {
        return asLongAs(Double.MAX_VALUE, condition, action);
    }

    /**
     * Execute a {@link Runnable} action until the {@code supplier}
     * {@link Predicate} returns false.
     *
     * @param predicate a {@link Predicate} that must return true in order
     *                  order for the method to continue executing. If the
     *                  predicate returns false, the method's execution
     *                  will terminate.
     * @param action    the action that will be run repeatedly.
     * @return {@code this}, used for method chaining.
     */
    public Voyager asLongAs(
        Predicate<Voyager> predicate,
        Runnable action
    ) {
        return asLongAs(Double.MAX_VALUE, predicate, action);
    }

    /*
     * the waitUntil and waitAsLongAs methods have to use busy waiting
     * because JDK8 doesn't support the Thread#onSpinWait method, which
     * is really obnoxious, but oh well... I guess...
     */

    /**
     * Pause until a certain condition is met.
     *
     * <p>
     * Unlike variants of {@link #tickUntil()}, the {@code waitUntil} method
     * and its derivatives DO NOT tick Pathfinder - they'll simply block the
     * current thread for a certain amount of time.
     * </p>
     *
     * @param condition the condition that must be met before continuing.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager waitUntil(Supplier<Boolean> condition) {
        return waitUntil(condition, Double.MAX_VALUE);
    }

    /**
     * Pause for a certain amount of time.
     *
     * <p>
     * Unlike variants of {@link #tickUntil()}, the {@code waitUntil} method
     * and its derivatives DO NOT tick Pathfinder - they'll simply block the
     * current thread for a certain amount of time.
     * </p>
     *
     * @param timeoutMs how long it should wait.
     * @return {@code this}, used for method chaining.
     */
    public Voyager waitUntil(double timeoutMs) {
        return waitUntil(() -> true, timeoutMs);
    }

    /**
     * Pause until a certain condition is met.
     *
     * <p>
     * Unlike variants of {@link #tickUntil()}, the {@code waitUntil} method
     * and its derivatives DO NOT tick Pathfinder - they'll simply block the
     * current thread for a certain amount of time.
     * </p>
     *
     * @param condition the condition that must be met before continuing.
     * @param maxTimeMs the maximum length of the pause. If the amount of
     *                  elapsed time exceeds this length, the condition will
     *                  break and Pathfinder will be unpaused, regardless of
     *                  whether the condition has been met.
     * @return this instance of Pathfinder, used for method chaining.
     */
    @SuppressWarnings("BusyWait")
    public Voyager waitUntil(Supplier<Boolean> condition, double maxTimeMs) {
        if (condition == null) throw new NullPointerException(
            "Attempted to use the waitUntil method with a null " +
            "condition supplier!"
        );

        if (maxTimeMs < 0) throw new InvalidTimeException(
            "Attempted to use an invalid time value! Make sure the " +
            "time value you're supplying is 0 or greater."
        );

        ElapsedTimer timer = new ElapsedTimer(true);

        try {
            while (
                !condition.get() && timer.isElapsedLessThan(maxTimeMs)
            ) Thread.sleep(Core.pathfinderWaitSleepTimeMs);
        } catch (InterruptedException ignored) {}

        return this;
    }

    /**
     * Pause as long as a certain condition is met. This method requires
     * doing just that the use of multiple threads, as this method has a
     * busy wait that will block the calling thread until {@code condition}'s
     * {@code get} returns false.
     *
     * <p>
     * Unlike variants of {@link #tickUntil()}, the {@code waitUntil} method
     * and its derivatives DO NOT tick Pathfinder - they'll simply block the
     * current thread for a certain amount of time.
     * </p>
     *
     * @param condition the condition that must be met in order to continue.
     *                  If this condition returns false, this method will
     *                  finish its execution and will unpause.
     * @return {@code this}, used for method chaining.
     */
    public Voyager waitAsLongAs(Supplier<Boolean> condition) {
        return waitAsLongAs(condition, Double.MAX_VALUE);
    }

    /**
     * Pause as long as a certain condition is met. This method requires
     * doing just that the use of multiple threads, as this method has a
     * busy wait that will block the calling thread until {@code condition}'s
     * {@code get} returns false.
     *
     * <p>
     * Unlike variants of {@link #tickUntil()}, the {@code waitUntil} method
     * and its derivatives DO NOT tick Pathfinder - they'll simply block the
     * current thread for a certain amount of time.
     * </p>
     *
     * @param condition the condition that must be met in order to continue.
     *                  If this condition returns false, this method will
     *                  finish its execution and will unpause.
     * @param maxTime   the maximum length of the pause. If the amount of
     *                  elapsed time exceeds this length, the condition will
     *                  break and Pathfinder will be unpaused, regardless of
     *                  whether the condition has been met.
     * @return {@code this}, used for method chaining.
     */
    @SuppressWarnings("BusyWait")
    public Voyager waitAsLongAs(
        Supplier<Boolean> condition,
        double maxTime
    ) {
        if (condition == null) throw new NullPointerException(
            "Attempted to use the waitAsLongAs method with a null " +
            "condition supplier!"
        );

        if (maxTime < 0) throw new InvalidTimeException(
            "Attempted to use an invalid time value! Make sure the " +
            "time value you're supplying is 0 or greater."
        );

        ElapsedTimer timer = new ElapsedTimer(true);

        try {
            while (
                condition.get() && timer.isElapsedLessThan(maxTime)
            ) Thread.sleep(Core.pathfinderWaitSleepTimeMs);
        } catch (InterruptedException ignored) {}

        return this;
    }

    /**
     * Follow a single trajectory from the global trajectory map.
     *
     * @param group the trajectory's group.
     * @param name  the name of the trajectory to follow.
     * @return {@code this}, used for method chaining.
     */
    public Voyager followTrajectory(String group, String name) {
        return followTrajectory(formatName(group, name));
    }

    /**
     * Follow a single trajectory from the global trajectory map.
     *
     * @param group the trajectory's group.
     * @param name  the name of the trajectory to follow.
     * @return {@code this}, used for method chaining.
     */
    public Voyager followTrajectory(Class<?> group, String name) {
        return followTrajectory(formatName(group.getSimpleName(), name));
    }

    /**
     * Follow a single trajectory from the global trajectory map. If the
     * trajectory cannot be found in the map, this method will throw an
     * exception indicating that the trajectory could not be found, and will
     * provide suggestions for what the user could have been trying to find.
     *
     * @param trajectoryName the name of the trajectory to follow.
     * @return {@code this}, used for method chaining.
     */
    public Voyager followTrajectory(final String trajectoryName) {
        if (TRAJECTORY_MAP.size() < 1) throw new IllegalStateException(
            "Cannot follow a trajectory " +
            "by name without first adding at least 1 trajectory " +
            "to the global trajectory map by using one of the " +
            "addTrajectory methods!"
        );

        // if the trajectory isn't contained in the list of trajectory names,
        // throw an exception containing a list of all of the potential names
        if (!TRAJECTORY_MAP.containsKey(trajectoryName)) {
            List<String> potentialNames = new ArrayList<>(10);

            TRAJECTORY_MAP.forEach(
                (name, trajectory) -> {
                    String lowercase = name.toLowerCase();

                    // if there's less than 8 trajectories, show all of them
                    // just to make debugging easier
                    if (
                        lowercase.contains(trajectoryName.toLowerCase())
                    ) potentialNames.add(name); else if (
                        TRAJECTORY_MAP.size() < 8
                    ) potentialNames.add(name);
                }
            );

            // if there are STILL no results, try only searching for the first
            // three letters of the trajectory's name - ideally, this should
            // use fuzzy finding, but this is such a minor feature i doubt
            // anyone is ever going to use and i'm only adding it because i
            // have free time during class and nothing better to be doing...
            if (potentialNames.size() == 0) {
                final String shortName;

                if (trajectoryName.length() > 3) shortName =
                    trajectoryName.substring(0, 3); else shortName =
                    trajectoryName;

                TRAJECTORY_MAP.forEach(
                    (name, trajectory) -> {
                        String lowercase = name.toLowerCase();

                        if (lowercase.contains(shortName)) potentialNames.add(
                            name
                        );
                    }
                );
            }

            StringBuilder builder = new StringBuilder(100);

            builder.append("Could not follow trajectory with name <%s>. ");
            builder.append("did you mean one of these? ");

            int size = potentialNames.size();
            for (int i = 0; i < size; i++) {
                String potentialName = potentialNames.get(i);

                builder.append(potentialName);

                if (i < size - 1) builder.append(", ");
            }

            throw new NullTrajectoryException(
                StringUtils.format(builder.toString(), trajectoryName)
            );
        }

        // if the trajectory is found, just follow it
        return followTrajectory(getTrajectory(trajectoryName));
    }

    /**
     * Follow a single trajectory.
     *
     * @param trajectory the trajectory to follow.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager followTrajectory(Trajectory trajectory) {
        if (trajectory == null) throw new NullPointerException(
            "Cannot follow a null trajectory!"
        );

        // this is really bad code but it's not important enough to take
        // the time to fix right now - maybe later?
        List<Trajectory> list = new ArrayList<Trajectory>(1) {

            {
                add(trajectory);
            }
        };

        followTrajectories(list);

        return this;
    }

    /**
     * Follow a single trajectory, after shifting it to the robot's
     * current position. This essentially converts an absolute trajectory
     * into a relative one.
     *
     * @param trajectory the trajectory to shift and follow.
     * @return {@code this}, used for method chaining.
     */
    public Voyager followRelativeTrajectory(Trajectory trajectory) {
        return followTrajectory(
            trajectory.shiftToRobot(new PointXYZ(0, 0, 0), getPosition())
        );
    }

    /**
     * Follow multiple trajectories.
     *
     * @param trajectories a list of trajectories to follow.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager followTrajectories(Trajectory... trajectories) {
        return followTrajectories(Arrays.asList(trajectories));
    }

    /**
     * Follow several trajectories after shifting them to the robot's
     * current position. This essentially converts an absolute trajectory
     * into a relative one.
     *
     * @param trajectories the trajectories to shift and follow.
     * @return {@code this}, used for method chaining.
     */
    public Voyager followRelativeTrajectories(Trajectory... trajectories) {
        for (Trajectory t : trajectories) followTrajectory(t);

        return this;
    }

    /**
     * Follow multiple trajectories.
     *
     * @param trajectories a list of trajectories to follow.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager followTrajectories(List<Trajectory> trajectories) {
        if (trajectories == null) throw new NullPointerException(
            "Cannot follow null trajectories!"
        );

        List<Follower> followers = new ArrayList<>();

        for (Trajectory trajectory : trajectories) followers.add(
            generator.generate(robot, trajectory)
        );

        follow(followers);

        return this;
    }

    /**
     * Follow a single follower.
     *
     * @param follower a single follower to follow.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager follow(Follower follower) {
        if (follower == null) throw new NullPointerException(
            "Attempted to follow a null Follower object - make sure " +
            "this object is not null."
        );

        executorManager.addExecutor(follower);

        return this;
    }

    /**
     * Follow a list of followers.
     *
     * @param followers a list of followers.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager follow(List<Follower> followers) {
        if (followers == null) throw new NullPointerException(
            "Attempted to follow a null list of Follower objects - " +
            "make sure the list you supply is not null."
        );

        executorManager.addExecutor(followers);

        return this;
    }

    /**
     * Follow a list of followers.
     *
     * @param followers a list of followers.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager follow(Follower... followers) {
        if (followers == null) throw new NullPointerException(
            "Attempted to follow a null list of Follower objects - " +
            "make sure the list you supply is not null."
        );

        executorManager.addExecutor(Arrays.asList(followers));

        return this;
    }

    /**
     * Go to a specific point. This method will create a new linear trajectory.
     *
     * @return {@code this}, used for method chaining.
     * @see #setSpeed(double)
     * @see #setTolerance(double)
     * @see #setAngleTolerance(Angle)
     */
    public Voyager goTo(PointXY point) {
        goTo(point.withHeading(getOdometry().getZ()));

        return this;
    }

    /**
     * Go to a specific point. This method will create a new linear trajectory.
     *
     * @param x the X coordinate to go to.
     * @param y the Y coordinate to go to.
     * @return {@code this}, used for method chaining.
     * @see #setSpeed(double)
     * @see #setTolerance(double)
     * @see #setAngleTolerance(Angle)
     */
    public Voyager goTo(double x, double y) {
        return goTo(new PointXY(x, y));
    }

    /**
     * Go to a specific point. This method will create a new linear trajectory.
     *
     * @param x        the X coordinate to go to.
     * @param y        the Y coordinate to go to.
     * @param zDegrees the Z coordinate to go to (in degrees).
     * @return {@code this}, used for method chaining.
     * @see #setSpeed(double)
     * @see #setTolerance(double)
     * @see #setAngleTolerance(Angle)
     */
    public Voyager goTo(double x, double y, double zDegrees) {
        return goTo(new PointXYZ(x, y, zDegrees));
    }

    /**
     * Go to a specific point. This method will create a new linear trajectory.
     *
     * @param x the X coordinate to go to.
     * @param y the Y coordinate to go to.
     * @param z the Z coordinate to go to.
     * @return {@code this}, used for method chaining.
     * @see #setSpeed(double)
     * @see #setTolerance(double)
     * @see #setAngleTolerance(Angle)
     */
    public Voyager goTo(double x, double y, Angle z) {
        return goTo(new PointXYZ(x, y, z));
    }

    private void checkForMissingDefaultValues() {
        // perform some wonderful exception checking...
        // this was a pretty big pain for me personally because I entirely
        // forgot that you actually need to set these values, so these lovely
        // and very handy reminders should definitely help... I think...
        // side note, I'm really craving some vanilla ice cream right now,
        // but I don't think I have any :(

        if (
            speed < 0 && tolerance < 0 && angleTolerance == null
        ) throw new RuntimeException(
            "Attempted to use the goTo method without having set " +
            "Pathfinder's default speed, tolerance, and angle " +
            "tolerance. Use the setSpeed(double), " +
            "setTolerance(double), and setAngleTolerance(Angle) " +
            "methods to set these values before using any  " +
            "variation of the goTo method."
        );

        if (speed < 0) throw new InvalidSpeedException(
            "Attempted to use the goTo method without having set the " +
            "speed of Pathfinder first! Use the setSpeed(double) " +
            "method to set a speed value."
        );

        if (tolerance < 0) throw new InvalidToleranceException(
            "Attempted to use the goTo method without having set the " +
            "tolerance of Pathfinder first! Use the setTolerance(double) " +
            "method to set a tolerance value."
        );

        if (angleTolerance == null) throw new NullAngleException(
            "Attempted to use the goTo method without having set the " +
            "angle tolerance of Pathfinder first! Use the" +
            "setAngleTolerance(Angle) method to set a " +
            "tolerance value."
        );
    }

    /**
     * Create a spline trajectory to a certain target point, and then follow
     * that aforementioned trajectory. This will use the default speed,
     * tolerance, and angle tolerance values.
     *
     * @param points a set of control points for the spline. This
     *               will automatically insert the robot's current
     *               position into this array. This array must have
     *               AT LEAST two points.
     * @return {@code this}, used for method chaining.
     */
    public Voyager splineTo(PointXYZ... points) {
        checkForMissingDefaultValues();

        return splineTo(speed, tolerance, angleTolerance, points);
    }

    /**
     * Use a {@link MultiSplineBuilder} to construct a spline trajectory.
     *
     * @param speed          the speed at which the robot should move. This
     *                       is a constant value.
     * @param tolerance      the tolerance used for determining whether the
     *                       robot is at the target point.
     * @param angleTolerance same thing as {@code tolerance}, but for the
     *                       robot's angle.
     * @param points         a set of control points for the spline. This
     *                       will automatically insert the robot's current
     *                       position into this array. This array must have
     *                       AT LEAST two points.
     * @return {@code this}, used for method chaining.
     */
    public Voyager multiSplineTo(
        double speed,
        double tolerance,
        Angle angleTolerance,
        PointXYZ... points
    ) {
        if (points.length < 2) throw new IllegalArgumentException(
            "At least two control points are required to use the " +
            "splineTo method."
        );
        checkForMissingDefaultValues();

        InvalidSpeedException.throwIfInvalid(
            "Invalid speed value provided! Speed must be between 0 and 1.",
            speed
        );
        InvalidToleranceException.throwIfInvalid(
            "Invalid tolerance! Tolerance must be a positive number.",
            tolerance
        );

        if (angleTolerance.deg() < 0) throw new InvalidToleranceException(
            "Invalid angle tolerance! " +
            "Angle tolerance must be greater than 0 degrees."
        );

        if (!Spline.areMonotonicX(points)) throw new SplineException(
            "Cannot create a spline with non-" +
            "monotonic X values! X values can only be either " +
            "increasing or decreasing, but not a combination of both."
        );

        double totalDistanceX = points[points.length - 1].distanceX(points[0]);

        double step =
            totalDistanceX /
            (points.length * Core.pathfinderSplineStepCoefficient);

        MultiSplineBuilder builder = new MultiSplineBuilder()
            .setDefaultSpeed(speed)
            .setDefaultTolerance(tolerance)
            .setDefaultAngleTolerance(angleTolerance)
            .setDefaultStep(step);

        for (PointXYZ point : points) builder.add(point, speed, step);

        Trajectory trajectory = builder.build();

        followTrajectory(trajectory);

        return this;
    }

    /**
     * Create a spline trajectory to a certain target point, and then follow
     * that aforementioned trajectory.
     *
     * <p>
     * If this method is called on a set of points with non-monotonic Y
     * values, this will instead invoke
     * {@link #multiSplineTo(double, double, Angle, PointXYZ...)}, which
     * supports non-monotonic Y values.
     * </p>
     *
     * @param speed          the speed at which the robot should move. This
     *                       is a constant value.
     * @param tolerance      the tolerance used for determining whether the
     *                       robot is at the target point.
     * @param angleTolerance same thing as {@code tolerance}, but for the
     *                       robot's angle.
     * @param points         a set of control points for the spline. This
     *                       will automatically insert the robot's current
     *                       position into this array. This array must have
     *                       AT LEAST two points.
     * @return {@code this}, used for method chaining.
     */
    public Voyager splineTo(
        double speed,
        double tolerance,
        Angle angleTolerance,
        PointXYZ... points
    ) {
        if (points.length < 2) throw new IllegalArgumentException(
            "At least two control points are required to use the " +
            "splineTo method."
        );

        InvalidSpeedException.throwIfInvalid(
            "Invalid speed value provided! Speed must be between 0 and 1.",
            speed
        );
        InvalidToleranceException.throwIfInvalid(
            "Invalid tolerance! Tolerance must be a positive number.",
            tolerance
        );

        if (angleTolerance.deg() < 0) throw new InvalidToleranceException(
            "Invalid angle tolerance! " +
            "Angle tolerance must be greater than 0 degrees."
        );

        NotNull.throwExceptionIfNull(
            "One or more points provided to splineTo was null!",
            (Object[]) points
        );

        // non-monotonic Y values means we need to use a multi spline instead
        if (!Spline.areMonotonicY(points)) return multiSplineTo(
            speed,
            tolerance,
            angleTolerance,
            points
        );

        // length is the total distance of the spline (NOT all control points)
        double length = PointXY.distance(points[0], points[points.length - 1]);

        // step should be relatively small - by default, it's 1/20th of
        // the spline's length.
        double step = length / Core.pathfinderStepDivisor;

        AdvancedSplineTrajectoryBuilder builder = new AdvancedSplineTrajectoryBuilder()
            .setSpeed(speed)
            .setTolerance(tolerance)
            .setAngleTolerance(angleTolerance)
            .setStep(step);

        PointXYZ robotPosition = getPosition();

        // if the first point is NOT the robot's current position, add it
        // to the trajectory
        if (!robotPosition.equals(points[0])) builder.add(robotPosition);

        PointXYZ lastPoint = points[0];
        for (int i = 1; i < points.length; i++) {
            PointXYZ point = points[i];

            if (point == null) throw new NullPointException(
                "Cannot use the splineTo method with a null " + "control point!"
            );

            if (!point.equals(lastPoint)) {
                builder.add(point);
                lastPoint = point;
            } else {
                throw new SplineException(
                    "There were duplicate adjacent points in the set " +
                    "of control points! This means there's the same " +
                    "point, twice in a row. The points were: " +
                    Arrays.toString(points)
                );
            }
        }

        Trajectory trajectory = builder.build();

        return followTrajectory(trajectory);
    }

    /**
     * Go to a specific point. This method will create a new linear trajectory.
     *
     * @param point the target point to go to.
     * @return this instance of Pathfinder, used for method chaining.
     * @see #setSpeed(double)
     * @see #setTolerance(double)
     * @see #setAngleTolerance(Angle)
     */
    public Voyager goTo(PointXYZ point) {
        NullPointException.throwIfInvalid(
            "Attempted to navigate to a null point.",
            point
        );

        checkForMissingDefaultValues();

        followTrajectory(
            new LinearTrajectory(point, speed, tolerance, angleTolerance)
        );

        return this;
    }

    /**
     * Go to a certain X value. This will use the robot's current position
     * in conjunction with a provided value to create a new point, and then
     * this will call the {@link #goTo(PointXYZ)} method, providing the
     * newly created point as a parameter.
     *
     * @param x the value to go to.
     * @return this instance of Pathfinder, used for method chaining.
     * @see #setSpeed(double)
     * @see #setTolerance(double)
     * @see #setAngleTolerance(Angle)
     */
    public Voyager goToX(double x) {
        ValidationUtils.validate(x, "x");

        return goTo(getPosition().withX(x));
    }

    /**
     * Go to a certain Y value. This will use the robot's current position
     * in conjunction with a provided value to create a new point, and then
     * this will call the {@link #goTo(PointXYZ)} method, providing the
     * newly created point as a parameter.
     *
     * @param y the value to go to.
     * @return this instance of Pathfinder, used for method chaining.
     * @see #setSpeed(double)
     * @see #setTolerance(double)
     * @see #setAngleTolerance(Angle)
     */
    public Voyager goToY(double y) {
        ValidationUtils.validate(y, "y");

        return goTo(getPosition().withY(y));
    }

    /**
     * Go to a certain Z value. This will use the robot's current position
     * in conjunction with a provided value to create a new point, and then
     * this will call the {@link #goTo(PointXYZ)} method, providing the
     * newly created point as a parameter.
     *
     * @param z the value to go to.
     * @return this instance of Pathfinder, used for method chaining.
     * @see #setSpeed(double)
     * @see #setTolerance(double)
     * @see #setAngleTolerance(Angle)
     */
    public Voyager goToZ(Angle z) {
        ValidationUtils.validate(z, "z");

        return goTo(getPosition().withZ(z));
    }

    /**
     * Create a new {@link TaskTrajectory} and add it to Pathfinder's
     * queue so that it can be executed
     *
     * @param initial    code to be executed the first time the trajectory's
     *                   {@code #isDone(PointXYZ)} method is called.
     * @param during     code to be executed any time the trajectory's
     *                   {@code #isDone(PointXYZ)} method is called.
     * @param onFinish   code to be executed whenever the task is finished.
     * @param isFinished a supplier that indicates if the task is finished.
     *                   If the task is not finished, it should continue stop
     *                   its execution.
     * @param minTimeMs  the minimum time, in milliseconds, the trajectory
     *                   will be active for.
     * @param maxTimeMs  the maximum time, in milliseconds, the trajectory
     *                   will be active for.
     * @return {@code this}, used for method chaining.
     */
    public Voyager task(
        Runnable initial,
        Runnable during,
        Runnable onFinish,
        Supplier<Boolean> isFinished,
        double minTimeMs,
        double maxTimeMs
    ) {
        Trajectory trajectory = new TaskTrajectoryBuilder()
            .setInitial(initial)
            .setDuring(during)
            .setOnFinish(onFinish)
            .setIsFinished(isFinished)
            .setMinTimeMs(minTimeMs)
            .setMaxTimeMs(maxTimeMs)
            .build();

        followTrajectory(trajectory);

        return this;
    }

    /**
     * Create a new {@link TaskTrajectory} and add it to Pathfinder's
     * queue so that it can be executed
     *
     * @param initial    code to be executed the first time the trajectory's
     *                   {@code #isDone(PointXYZ)} method is called.
     * @param during     code to be executed any time the trajectory's
     *                   {@code #isDone(PointXYZ)} method is called.
     * @param onFinish   code to be executed whenever the task is finished.
     * @param isFinished a supplier that indicates if the task is finished.
     *                   If the task is not finished, it should continue stop
     *                   its execution.
     * @return {@code this}, used for method chaining.
     */
    public Voyager task(
        Runnable initial,
        Runnable during,
        Runnable onFinish,
        Supplier<Boolean> isFinished
    ) {
        return task(initial, during, onFinish, isFinished, 0, Double.MAX_VALUE);
    }

    /**
     * Create a new {@link TaskTrajectory} and add it to Pathfinder's
     * queue so that it can be executed
     *
     * @param during     code to be executed any time the trajectory's
     *                   {@code #isDone(PointXYZ)} method is called.
     * @param isFinished a supplier that indicates if the task is finished.
     *                   If the task is not finished, it should continue stop
     *                   its execution.
     * @return {@code this}, used for method chaining.
     */
    public Voyager task(Runnable during, Supplier<Boolean> isFinished) {
        return task(() -> {}, during, () -> {}, isFinished);
    }

    /**
     * Move the robot in a certain direction for a certain amount of time.
     * This is a blocking method call, meaning it will block the current thread
     * until its execution has finished (whatever {@code timeoutMs} is).
     *
     * @param translation the translation that will be set to the robot.
     *                    This value may not be null.
     * @param timeoutMs   how long the robot should move for. This value is
     *                    represented in milliseconds and must be greater
     *                    than 0. This value may also not be infinite.
     * @return {@code this, used for method chaining}
     */
    @SuppressWarnings("BusyWait")
    public Voyager moveFor(Translation translation, double timeoutMs) {
        if (translation == null) throw new NullPointerException(
            "Cannot use a null translation!"
        );

        if (
            timeoutMs <= 0 ||
            Double.isInfinite(timeoutMs) ||
            timeoutMs == Double.MAX_VALUE
        ) throw new IllegalArgumentException("Invalid timeout!");

        ElapsedTimer timer = new ElapsedTimer(true);
        setTranslation(translation);

        try {
            while (timer.isElapsedLessThan(timeoutMs)) Thread.sleep(
                Core.pathfinderWaitSleepTimeMs
            );
        } catch (Exception ignored) {}

        return this;
    }

    /**
     * Move the robot in a certain direction (specified by a translation)
     * for a specified amount of time, in milliseconds. This is a blocking
     * method call, meaning it will block the current thread until its
     * execution has finished (whatever {@code timeoutMs} is).
     *
     * @param vx        the vx component of the robot's translation.
     * @param vy        the vy component of the robot's translation.
     * @param vz        the vz component of the robot's translation.
     * @param timeoutMs how long the robot should move for, in milliseconds.
     * @return {@code this}, used for method chaining.
     */
    public Voyager moveFor(
        double vx,
        double vy,
        double vz,
        double timeoutMs
    ) {
        return moveFor(new Translation(vx, vy, vz), timeoutMs);
    }

    /**
     * Get Pathfinder's {@code Stopwatch}.
     *
     * @return Pathfinder's {@code Stopwatch} instance.
     */
    public Stopwatch stopwatch() {
        return this.stopwatch;
    }

    /**
     * Is Pathfinder currently active? Pathfinder is considered to be active
     * if there is at least 1 active follower in Pathfinder's
     * {@link ExecutorManager}. If there is 1 or more followers, this method
     * will return true. If there are not any followers, this method will
     * return false, indicating that Pathfinder is currently idle and is not
     * in the process of following a {@link Follower}.
     *
     * @return is Pathfinder currently active? This method will return true
     * if Pathfinder is active (meaning its currently following a path) and
     * will return false if Pathfinder is not active (meaning its idle).
     */
    public boolean isActive() {
        return executorManager.isActive();
    }

    /**
     * Get this instance of {@code Pathfinder}'s {@link ExecutorManager}.
     *
     * @return Pathfinder's executor manager.
     */
    public ExecutorManager getExecutorManager() {
        return executorManager;
    }

    /**
     * Clear the {@link ExecutorManager}, resetting just about everything.
     *
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager clear() {
        this.pluginManager.preClear(this);
        this.executorManager.clearExecutors();
        this.pluginManager.onClear(this);

        return this;
    }

    /**
     * Use the {@link #getOdometry()} method in combination with the
     * odometry class' {@link Odometry#getPosition()} to access the
     * robot's current position.
     *
     * @return the robot's current position.
     */
    public PointXYZ getPosition() {
        return getOdometry().getPosition();
    }

    /**
     * Get the robot's raw position.
     *
     * @return the robot's raw position.
     */
    public PointXYZ getRawPosition() {
        return getOdometry().getRawPosition();
    }

    /**
     * Get the robot's current translation.
     *
     * @return the robot's current translation.
     */
    public Translation getTranslation() {
        Translation translation = getDrive().getTranslation();

        return translation != null
            ? translation
            : Core.pathfinderDefaultTranslation;
    }

    /**
     * Set a translation to the robot. This is how to manually move your robot.
     * If, for example, you're in TeleOp, and you'd like to drive your robot
     * according to some joystick inputs, this is the method you should use.
     *
     * <p>
     * Calling this method will immediately update the robot's translation.
     * However, if the robot is still under the control of a trajectory
     * or follower or executor, the translation you set will have next to
     * no effect. As soon as the follower/trajectory/executor is ticked again
     * with the {@link #tick()} method, the translation will be set to whatever
     * the {@link Follower} says the translation should be.
     * </p>
     *
     * @param translation the translation to set to the robot. This translation
     *                    should be RELATIVE, meaning forwards is forwards for
     *                    the robot, not forwards relative to you.
     * @return this instance of Pathfinder, used for method chaining.
     */
    public Voyager setTranslation(Translation translation) {
        if (translation == null) throw new NullPointerException(
            "Attempted to use the setTranslation method, but provided " +
            "a null translation - make sure this translation " +
            "isn't null next time, alright? Cool."
        );

        getDrive().setTranslation(translation);

        return this;
    }

    /**
     * Get the X component of the robot's translation.
     *
     * @return the X component of the robot's translation.
     */
    public double getVx() {
        return getTranslation().vx();
    }

    /**
     * Set the X component of the robot's translation.
     *
     * <p>
     * This will update the robot's translation by copying over unchanged
     * values and reassigning the changed value.
     * </p>
     *
     * @param vx the X component of the robot's translation.
     * @return {@code this}, used for method chaining.
     */
    public Voyager setVx(double vx) {
        return setTranslation(new Translation(vx, getVy(), getVz()));
    }

    /**
     * Get the Y component of the robot's translation.
     *
     * @return the Y component of the robot's translation.
     */
    public double getVy() {
        return getTranslation().vy();
    }

    /**
     * Set the Y component of the robot's translation.
     *
     * <p>
     * This will update the robot's translation by copying over unchanged
     * values and reassigning the changed value.
     * </p>
     *
     * @param vy the Y component of the robot's translation.
     * @return {@code this}, used for method chaining.
     */
    public Voyager setVy(double vy) {
        return setTranslation(new Translation(getVx(), vy, getVz()));
    }

    /**
     * Get the Z component of the robot's translation.
     *
     * @return the Z component of the robot's translation.
     */
    public double getVz() {
        return getTranslation().vz();
    }

    /**
     * Set the Z component of the robot's translation.
     *
     * <p>
     * This will update the robot's translation by copying over unchanged
     * values and reassigning the changed value.
     * </p>
     *
     * @param vz the Z component of the robot's translation.
     * @return {@code this}, used for method chaining.
     */
    public Voyager setVz(double vz) {
        return setTranslation(new Translation(getVx(), getVy(), vz));
    }

    /**
     * Get how long the current follower has been executing. If no followers
     * have executed, this will return 0. If no followers are active, but
     * a follower has been active in the past, this will return the execution
     * time of the last follower.
     *
     * @return the execution time of the current follower.
     */
    public double getExecutionTime() {
        return getExecutorManager().getExecutionTime();
    }

    /**
     * Get the robot's velocity according to the last motion snapshot. If
     * no motion snapshots have been taken, invoking this method will cause
     * a {@link NullPointerException}.
     *
     * <p>
     * Along with all other profiler-related methods, the return value of
     * this method is units per second. Because units are not specified
     * anywhere and are thus left up to the user, these units can be anything
     * at all - but the time will always remain in seconds.
     * </p>
     *
     * @return the robot's velocity, in units per second.
     */
    public Velocity getVelocity() {
        return profiler.getLastSnapshot().getVelocity();
    }

    /**
     * Get the robot's velocity according to the last motion snapshot. If
     * no motion snapshots have been taken, invoking this method will cause
     * a {@link NullPointerException}.
     *
     * <p>
     * Along with all other profiler-related methods, the return value of
     * this method is units per second. Because units are not specified
     * anywhere and are thus left up to the user, these units can be anything
     * at all - but the time will always remain in seconds.
     * </p>
     *
     * @return the robot's velocity, in units per second.
     */
    public double getVelocityXY() {
        return profiler.getLastSnapshot().getVelocityXY();
    }

    /**
     * Get the robot's X velocity according to the last motion snapshot. If
     * no motion snapshots have been taken, invoking this method will cause
     * a {@link NullPointerException}.
     *
     * <p>
     * Along with all other profiler-related methods, the return value of
     * this method is units per second. Because units are not specified
     * anywhere and are thus left up to the user, these units can be anything
     * at all - but the time will always remain in seconds.
     * </p>
     *
     * @return the robot's X velocity, in units per second.
     */
    public double getVelocityX() {
        return profiler.getLastSnapshot().getVelocityX();
    }

    /**
     * Get the robot's Y velocity according to the last motion snapshot. If
     * no motion snapshots have been taken, invoking this method will cause
     * a {@link NullPointerException}.
     *
     * <p>
     * Along with all other profiler-related methods, the return value of
     * this method is units per second. Because units are not specified
     * anywhere and are thus left up to the user, these units can be anything
     * at all - but the time will always remain in seconds.
     * </p>
     *
     * @return the robot's Y velocity, in units per second.
     */
    public double getVelocityY() {
        return profiler.getLastSnapshot().getVelocityY();
    }

    /**
     * Get the robot's Z velocity according to the last motion snapshot. If
     * no motion snapshots have been taken, invoking this method will cause
     * a {@link NullPointerException}.
     *
     * <p>
     * Along with all other profiler-related methods, the return value of
     * this method is units per second. Because units are not specified
     * anywhere and are thus left up to the user, these units can be anything
     * at all - but the time will always remain in seconds.
     * </p>
     *
     * @return the robot's Z velocity, in units per second. Because this is
     * an angle and not a degree or radian measure, I can't think of a better
     * term, but it's basically just angle per second.
     */
    public Angle getVelocityZ() {
        return profiler.getLastSnapshot().getVelocityZ();
    }

    /**
     * Get the robot's acceleration according to the last motion snapshot. If
     * no motion snapshots have been taken, invoking this method will cause
     * a {@link NullPointerException}.
     *
     * <p>
     * Along with all other profiler-related methods, the return value of
     * this method is units per second. Because units are not specified
     * anywhere and are thus left up to the user, these units can be anything
     * at all - but the time will always remain in seconds.
     * </p>
     *
     * @return the robot's acceleration, in units per second squared.
     */
    public double getAccelerationXY() {
        return profiler.getLastSnapshot().getAccelerationXY();
    }

    /**
     * Get the robot's X acceleration according to the last motion snapshot. If
     * no motion snapshots have been taken, invoking this method will cause
     * a {@link NullPointerException}.
     *
     * <p>
     * Along with all other profiler-related methods, the return value of
     * this method is units per second. Because units are not specified
     * anywhere and are thus left up to the user, these units can be anything
     * at all - but the time will always remain in seconds.
     * </p>
     *
     * @return the robot's X acceleration, in units per second squared.
     */
    public double getAccelerationX() {
        return profiler.getLastSnapshot().getAccelerationX();
    }

    /**
     * Get the robot's Y acceleration according to the last motion snapshot. If
     * no motion snapshots have been taken, invoking this method will cause
     * a {@link NullPointerException}.
     *
     * <p>
     * Along with all other profiler-related methods, the return value of
     * this method is units per second. Because units are not specified
     * anywhere and are thus left up to the user, these units can be anything
     * at all - but the time will always remain in seconds.
     * </p>
     *
     * @return the robot's Y acceleration, in units per second squared.
     */
    public double getAccelerationY() {
        return profiler.getLastSnapshot().getAccelerationY();
    }

    /**
     * Get the robot's Z acceleration according to the last motion snapshot. If
     * no motion snapshots have been taken, invoking this method will cause
     * a {@link NullPointerException}.
     *
     * <p>
     * Along with all other profiler-related methods, the return value of
     * this method is units per second. Because units are not specified
     * anywhere and are thus left up to the user, these units can be anything
     * at all - but the time will always remain in seconds.
     * </p>
     *
     * @return the robot's Z acceleration, in units per second squared.
     */
    public Angle getAccelerationZ() {
        return profiler.getLastSnapshot().getAccelerationZ();
    }

    /**
     * Lock Pathfinder's heading by using a modifier. Every time a translation
     * is set to the drive train, it will be modified so that it will turn
     * the robot towards the provided heading value.
     *
     * <p>
     * In order to reverse this effect, use {@link #unlockHeading()}.
     * </p>
     *
     * <p>
     * The following three methods CAN NOT be combined or you will encounter
     * some issues: (combined meaning nested)
     * <ul>
     *     <li>{@link #lockHeading(Angle)}</li>
     *     <li>{@link #lockHeading(PointXY)}</li>
     *     <li>{@link #lockHeading(PointXY, Angle)}</li>
     * </ul>
     * Basically, this is a technical limitation because of my laziness and
     * poorly written code. This might get fixed at some point in the near
     * future, but it's not a high priority.
     * </p>
     *
     * @param heading the heading Pathfinder should remain at.
     * @return {@code this}, used for method chaining.
     */
    public Voyager lockHeading(Angle heading) {
        Function<Translation, Translation> modifier = new HeadingLock(
            heading,
            turnController,
            this.getPosition()::z
        );

        lastDriveModifier = getDrive().getDriveModifier();
        getDrive().setDriveModifier(modifier);

        return this;
    }

    /**
     * Lock Pathfinder's heading by making the robot face a given point. The
     * robot will always face directly towards the provided point.
     *
     * <p>
     * In order to reverse this effect, use {@link #unlockHeading()}.
     * </p>
     *
     * <p>
     * The following three methods CAN NOT be combined or you will encounter
     * some issues: (combined meaning nested)
     * <ul>
     *     <li>{@link #lockHeading(Angle)}</li>
     *     <li>{@link #lockHeading(PointXY)}</li>
     *     <li>{@link #lockHeading(PointXY, Angle)}</li>
     * </ul>
     * Basically, this is a technical limitation because of my laziness and
     * poorly written code. This might get fixed at some point in the near
     * future, but it's not a high priority.
     * </p>
     *
     * @param point the point to lock heading around.
     * @return {@code this}, used for method chaining.
     */
    public Voyager lockHeading(PointXY point) {
        Function<Translation, Translation> modifier = new AutoRotator(
            this,
            turnController,
            point,
            Angle.DEG_0,
            true
        );

        lastDriveModifier = getDrive().getDriveModifier();
        getDrive().setDriveModifier(modifier);

        return this;
    }

    /**
     * Lock Pathfinder's heading by making the robot face a given point. The
     * robot will always face directly towards the provided point, plus
     * the provided angle offset.
     *
     * <p>
     * In order to reverse this effect, use {@link #unlockHeading()}.
     * </p>
     *
     * <p>
     * The following three methods CAN NOT be combined or you will encounter
     * some issues: (combined meaning nested)
     * <ul>
     *     <li>{@link #lockHeading(Angle)}</li>
     *     <li>{@link #lockHeading(PointXY)}</li>
     *     <li>{@link #lockHeading(PointXY, Angle)}</li>
     * </ul>
     * Basically, this is a technical limitation because of my laziness and
     * poorly written code. This might get fixed at some point in the near
     * future, but it's not a high priority.
     * </p>
     *
     * @param point       the point to lock heading around.
     * @param angleOffset the offset to be applied to the angle Pathfinder
     *                    determines it needs to face.
     * @return {@code this}, used for method chaining.
     */
    public Voyager lockHeading(PointXY point, Angle angleOffset) {
        Function<Translation, Translation> modifier = new AutoRotator(
            this,
            turnController,
            point,
            angleOffset,
            true
        );

        lastDriveModifier = getDrive().getDriveModifier();
        getDrive().setDriveModifier(modifier);

        return this;
    }

    /**
     * Unlock Pathfinder's heading.
     *
     * @return {@code this}, used for method chaining.
     */
    public Voyager unlockHeading() {
        getDrive().setDriveModifier(lastDriveModifier);

        return this;
    }

    /**
     * Create a new {@code Button} (sort of, anyways).
     *
     * @param supplier  a supplier that returns the system's state.
     * @param predicate a predicate that evaluates the system's state.
     * @return a new {@code Button}.
     */
    public <T> Button newButton(Supplier<T> supplier, Predicate<T> predicate) {
        return new Button(listenerManager, supplier, predicate);
    }

    /**
     * Create a new {@code Button} with a pre-attached {@code ListenerManager}.
     *
     * @param stateSupplier a supplier that returns the button's state. If
     *                      the button is pressed, this should return true.
     *                      Otherwise, this should return false.
     * @return a new {@code Button}.
     */
    public Button newButton(Supplier<Boolean> stateSupplier) {
        return new Button(listenerManager, stateSupplier);
    }

    /**
     * Create a new {@code Button} with a pre-attached {@code ListenerManager}.
     * This button is based on a trigger.
     *
     * @param stateSupplier a supplier that returns the trigger's state. This
     *                      value should almost always be within the range
     *                      of 0-1, where 0 is not at all pressed and 1 is
     *                      completely pressed.
     * @param deadZone      the minimum value of the trigger that counts as
     *                      being pressed.
     * @return a new {@code Button}.
     */
    public Button newTriggerButton(
        Supplier<Double> stateSupplier,
        double deadZone
    ) {
        return new Button(
            listenerManager,
            () -> stateSupplier.get() >= deadZone
        );
    }

    @Override
    public int hashCode() {
        return robot.hashCode() + generator.hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        return false;
    }

    @Override
    protected Object clone() throws CloneNotSupportedException {
        throw new CloneNotSupportedException();
    }

    /**
     * Convert this instance of {@code Pathfinder} into a {@code String}.
     * Really simply, this just return's the current position. To be
     * completely honest, I have absolutely no idea why anybody would ever
     * even want to do this, but I figured I'd include it anyways. No harm
     * in that, I guess... right?
     *
     * @return the current position, as a string.
     */
    @Override
    public String toString() {
        return getPosition().toString();
    }
}
