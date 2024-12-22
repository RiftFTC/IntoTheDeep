package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TimeSys extends SubsystemBase {
    public enum GameState {
        INIT,
        AUTO,
        TELEOP,
        ENDGAME,
        HANG_PERIOD,
        FINISHED
    }

    private boolean isStarted = false;
    private final ElapsedTime timer;

    public GameState gameState = GameState.INIT;

    public TimeSys() {
        timer = new ElapsedTime();
    }

    public void setGameState(GameState gameState) {
        this.gameState = gameState;
    }

    public GameState getGameState() {
        return gameState;
    }

    @Override
    public void periodic() {
        if (!isStarted) {
            timer.reset();
            isStarted = true;
            gameState = GameState.TELEOP;
        }
        if (timer.seconds() > 120) {
            gameState = GameState.FINISHED;
        } else if (timer.seconds() > 110) {
            gameState = GameState.HANG_PERIOD;
        } else if (timer.seconds() > 90) {
            gameState = GameState.ENDGAME;
        }
    }
}
