package org.firstinspires.ftc.teamcode.util;

import android.util.Log;
import com.google.gson.Gson;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;

public class GamepadServer {

    private Thread gamepadThread;

    public GamepadServer(Gamepad gamepad) {
        gamepadThread = new Thread(() -> GamepadThread.init(gamepad));
        gamepadThread.start();
    }

    public void shutdown() {
        GamepadThread.shutdown();
    }

    static class GamepadThread {
        private static Gamepad gamepad;
        private static ServerSocket serverSocket;
        private static Socket socket;
        private static BufferedReader in;

        // Add a flag to check if the server is active or should shut down
        private static boolean running = true;

        public static void init(Gamepad gamepad) {
            try {
                serverSocket = new ServerSocket(12345); // Port 12345
                Log.i("GamepadServer", "Server started!");
                socket = serverSocket.accept();
                Log.i("GamepadServer", "Client connected!");

                in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                GamepadThread.gamepad = gamepad;

                String line;
                while (running && (line = in.readLine()) != null) {
                    RecievedGamepadState.Gamepad receivedGamepad = parseGamepadData(line);
                    synchronized (gamepad) {
                        copyIntoSdkGamepad(receivedGamepad, gamepad);
                    }
                }

                shutdown();

            } catch (IOException e) {
                if (running) {
                    e.printStackTrace();
                    Log.e("GamepadServer", "Error during server operation: " + e.getMessage());
                }
            }
        }

        public static void copyIntoSdkGamepad(RecievedGamepadState.Gamepad src, Gamepad dst) {
            dst.left_stick_x = src.left_stick_x;
            dst.left_stick_y = src.left_stick_y;
            dst.right_stick_x = src.right_stick_x;
            dst.right_stick_y = src.right_stick_y;
            dst.dpad_up = src.dpad_up;
            dst.dpad_down = src.dpad_down;
            dst.dpad_left = src.dpad_left;
            dst.dpad_right = src.dpad_right;
            dst.a = src.a;
            dst.b = src.b;
            dst.x = src.x;
            dst.y = src.y;
            dst.guide = src.guide;
            dst.start = src.start;
            dst.back = src.back;
            dst.left_bumper = src.left_bumper;
            dst.right_bumper = src.right_bumper;
            dst.left_stick_button = src.left_stick_button;
            dst.right_stick_button = src.right_stick_button;
            dst.left_trigger = src.left_trigger;
            dst.right_trigger = src.right_trigger;
            dst.touchpad = src.touchpad;
        }

        public static RecievedGamepadState.Gamepad parseGamepadData(String data) {
            Gson gson = new Gson();
            return gson.fromJson(data, RecievedGamepadState.Gamepad.class);
        }

        public static void shutdown() {
            running = false;
            try {
                if (in != null) {
                    in.close();
                }
                if (socket != null && !socket.isClosed()) {
                    socket.close();
                }
                if (serverSocket != null && !serverSocket.isClosed()) {
                    serverSocket.close();
                }
                Log.i("GamepadServer", "Server shut down successfully.");
            } catch (IOException e) {
                Log.e("GamepadServer", "Error during server shutdown: " + e.getMessage());
            }
        }
    }
}
