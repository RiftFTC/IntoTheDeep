/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.kinematics;

import java.util.function.Supplier;
import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.Translation;

/**
 * The simplest form of swerve drive kinematics. This class operates very
 * simply - each of the four swerve modules has its own {@link RelativeSwerveModuleKinematics}
 * instance responsible for controlling the module's angle and power. The
 * only thing this class does differently is incorporating turn values.
 *
 * <p>
 * Unlike other popular swerve drive implementations, turning is not handled
 * by rotating the wheels - rather, turning is handled by adjusting the power
 * of each of the wheels. This, functionally, doesn't have very much of
 * a difference - it does make it a bit more challenging to do complex math
 * related to the robot's center of rotation and all that, but it works
 * quite simply and quite well.
 * </p>
 *
 * @author Colin Robertson
 * @since 0.0.0
 */
public class RelativeSwerveDriveKinematics
    implements Kinematics<RelativeSwerveState> {
    /**
     * The front right module's kinematics.
     */
    private final RelativeSwerveModuleKinematics frontRightKinematics;

    /**
     * The front left module's kinematics.
     */
    private final RelativeSwerveModuleKinematics frontLeftKinematics;

    /**
     * The back right module's kinematics.
     */
    private final RelativeSwerveModuleKinematics backRightKinematics;

    /**
     * The back left module's kinematics.
     */
    private final RelativeSwerveModuleKinematics backLeftKinematics;

    /**
     * The front right module's angle.
     * When the {@link Supplier#get()} method is called on this, this should
     * return the module's angle, 0 degrees being straight forwards.
     */
    private final Supplier<Angle> frontRightModuleAngle;

    /**
     * The front left module's angle.
     * When the {@link Supplier#get()} method is called on this, this should
     * return the module's angle, 0 degrees being straight forwards.
     */
    private final Supplier<Angle> frontLeftModuleAngle;

    /**
     * The back right module's angle.
     * When the {@link Supplier#get()} method is called on this, this should
     * return the module's angle, 0 degrees being straight forwards.
     */
    private final Supplier<Angle> backRightModuleAngle;

    /**
     * The back left module's angle.
     * When the {@link Supplier#get()} method is called on this, this should
     * return the module's angle, 0 degrees being straight forwards.
     */
    private final Supplier<Angle> backLeftModuleAngle;

    /**
     * The chassis' turn multiplier. See the documentation provided in this
     * class' constructor to learn more.
     */
    private final double turnMultiplier;

    /**
     * Create a new instance of the relative swerve drive kinematics class
     * by using the same module kinematics for each of the swerve modules.
     * This, in almost all cases, should function exactly the same as if
     * every module had their own kinematics. The only difference is that
     * this constructor requires three fewer parameters.
     *
     * @param moduleKinematics      the swerve module kinematics for all four
     *                              of the chassis' modules.
     * @param frontRightModuleAngle the front right module's angle.
     * @param frontLeftModuleAngle  the front left module's angle.
     * @param backRightModuleAngle  the back right module's angle.
     * @param backLeftModuleAngle   the back left module angle.
     * @param turnMultiplier        the chassis' turn multiplier. Having a
     *                              higher turn multiplier means the robot
     *                              will turn faster. Having a lower turn
     *                              multiplier will mean the robot will turn
     *                              slower. Typically, this value should be
     *                              somewhere between 0.5 and 1 - but you'll
     *                              have to do some testing and figure out
     *                              what works best for you. Also, having
     *                              a negative turn multiplier will mean the
     *                              chassis will turn in the opposite direction.
     */
    public RelativeSwerveDriveKinematics(
        RelativeSwerveModuleKinematics moduleKinematics,
        Supplier<Angle> frontRightModuleAngle,
        Supplier<Angle> frontLeftModuleAngle,
        Supplier<Angle> backRightModuleAngle,
        Supplier<Angle> backLeftModuleAngle,
        double turnMultiplier
    ) {
        this(
            moduleKinematics,
            moduleKinematics,
            moduleKinematics,
            moduleKinematics,
            frontRightModuleAngle,
            frontLeftModuleAngle,
            backRightModuleAngle,
            backLeftModuleAngle,
            turnMultiplier
        );
    }

    /**
     * Create a new instance of the relative swerve drive kinematics class.
     *
     * @param frontRightKinematics  the front right module's kinematics.
     * @param frontLeftKinematics   the front left module's kinematics.
     * @param backRightKinematics   the back right module's kinematics.
     * @param backLeftKinematics    the back left module's kinematics.
     * @param frontRightModuleAngle the front right module's angle.
     * @param frontLeftModuleAngle  the front left module's angle.
     * @param backRightModuleAngle  the back right module's angle.
     * @param backLeftModuleAngle   the back left module angle.
     * @param turnMultiplier        the chassis' turn multiplier. Having a
     *                              higher turn multiplier means the robot
     *                              will turn faster. Having a lower turn
     *                              multiplier will mean the robot will turn
     *                              slower. Typically, this value should be
     *                              somewhere between 0.5 and 1 - but you'll
     *                              have to do some testing and figure out
     *                              what works best for you. Also, having
     *                              a negative turn multiplier will mean the
     *                              chassis will turn in the opposite direction.
     */
    public RelativeSwerveDriveKinematics(
        RelativeSwerveModuleKinematics frontRightKinematics,
        RelativeSwerveModuleKinematics frontLeftKinematics,
        RelativeSwerveModuleKinematics backRightKinematics,
        RelativeSwerveModuleKinematics backLeftKinematics,
        Supplier<Angle> frontRightModuleAngle,
        Supplier<Angle> frontLeftModuleAngle,
        Supplier<Angle> backRightModuleAngle,
        Supplier<Angle> backLeftModuleAngle,
        double turnMultiplier
    ) {
        this.frontRightKinematics = frontRightKinematics;
        this.frontLeftKinematics = frontLeftKinematics;
        this.backRightKinematics = backRightKinematics;
        this.backLeftKinematics = backLeftKinematics;

        this.frontRightModuleAngle = frontRightModuleAngle;
        this.frontLeftModuleAngle = frontLeftModuleAngle;
        this.backRightModuleAngle = backRightModuleAngle;
        this.backLeftModuleAngle = backLeftModuleAngle;

        this.turnMultiplier = turnMultiplier;
    }

    /**
     * Create a swerve drive chassis state based on a translation.
     *
     * @param translation the translation to create a state for.
     * @return a created relative swerve drive state based
     * on the provided translation.
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public RelativeSwerveState calculate(Translation translation) {
        Angle angle = translation.angle();

        double vz = translation.vz() * turnMultiplier;

        double frontRightTurn = frontRightKinematics.calculate(
            frontRightModuleAngle.get(),
            angle.add(Angle.fromDeg(vz))
        );
        double frontLeftTurn = frontLeftKinematics.calculate(
            frontLeftModuleAngle.get(),
            angle.add(Angle.fromDeg(-vz))
        );
        double backRightTurn = backRightKinematics.calculate(
            backRightModuleAngle.get(),
            angle.add(Angle.fromDeg(vz))
        );
        double backLeftTurn = backLeftKinematics.calculate(
            backLeftModuleAngle.get(),
            angle.add(Angle.fromDeg(-vz))
        );

        double frontRightDrive = translation.magnitude();
        double frontLeftDrive = translation.magnitude();
        double backRightDrive = translation.magnitude();
        double backLeftDrive = translation.magnitude();

        RelativeSwerveModuleState frontRightState = new RelativeSwerveModuleState(
            frontRightTurn,
            frontRightDrive
        );
        RelativeSwerveModuleState frontLeftState = new RelativeSwerveModuleState(
            frontLeftTurn,
            frontLeftDrive
        );
        RelativeSwerveModuleState backRightState = new RelativeSwerveModuleState(
            backRightTurn,
            backRightDrive
        );
        RelativeSwerveModuleState backLeftState = new RelativeSwerveModuleState(
            backLeftTurn,
            backLeftDrive
        );

        return new RelativeSwerveState(
            frontRightState,
            frontLeftState,
            backRightState,
            backLeftState
        );
    }

    @Override
    public Translation toTranslation(RelativeSwerveState state) {
        throw new RuntimeException(
            "Cannot convert a swerve state to a translation " +
            "using the relative swerve kinematics."
        );
    }

    /**
     * Calculate an optimized swerve state based on a translation.
     *
     * <p>
     * My left hand feels like it's about to fall off from typing this much,
     * so I'm just gonna ask you to go read the documentation in the
     * swerve module state class about optimization to figure it out.
     * Good luck!
     * </p>
     *
     * @param translation the translation the robot should follow.
     * @return an optimized swerve state according to that translation.
     */
    @SuppressWarnings("DuplicatedCode")
    public RelativeSwerveState calculateOptimized(Translation translation) {
        Angle angle = translation.angle();

        double vz = translation.vz() * turnMultiplier;

        double frontRightTurn = frontRightKinematics.calculate(
            frontRightModuleAngle.get(),
            angle.add(Angle.fromDeg(vz))
        );
        double frontLeftTurn = frontLeftKinematics.calculate(
            frontLeftModuleAngle.get(),
            angle.add(Angle.fromDeg(-vz))
        );
        double backRightTurn = backRightKinematics.calculate(
            backRightModuleAngle.get(),
            angle.add(Angle.fromDeg(vz))
        );
        double backLeftTurn = backLeftKinematics.calculate(
            backLeftModuleAngle.get(),
            angle.add(Angle.fromDeg(-vz))
        );

        double frontRightDrive = translation.magnitude();
        double frontLeftDrive = translation.magnitude();
        double backRightDrive = translation.magnitude();
        double backLeftDrive = translation.magnitude();

        RelativeSwerveModuleState frontRightState = RelativeSwerveModuleState.optimized(
            angle,
            frontRightDrive,
            frontRightModuleAngle.get(),
            frontRightKinematics
        );
        RelativeSwerveModuleState frontLeftState = RelativeSwerveModuleState.optimized(
            angle,
            frontLeftDrive,
            frontLeftModuleAngle.get(),
            frontLeftKinematics
        );
        RelativeSwerveModuleState backRightState = RelativeSwerveModuleState.optimized(
            angle,
            backRightDrive,
            backRightModuleAngle.get(),
            backRightKinematics
        );
        RelativeSwerveModuleState backLeftState = RelativeSwerveModuleState.optimized(
            angle,
            backLeftDrive,
            backLeftModuleAngle.get(),
            backLeftKinematics
        );

        return new RelativeSwerveState(
            frontRightState,
            frontLeftState,
            backRightState,
            backLeftState
        );
    }
}
