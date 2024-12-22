package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.hardware.SimpleServo;

public class ServoProfiled {
    private final SimpleServo servo;
    public AsymmetricMotionProfile profile;
    private boolean isProfiled = false;

    private final boolean async;

    public ServoProfiled(SimpleServo servo, boolean reversed, double initialPosition){
        this.servo = servo;
        this.async = false;
        if(reversed) this.servo.setInverted(reversed);
        setInitialPosition(initialPosition);
    }

    public ServoProfiled(SimpleServo servo, boolean reversed, double profileMaxVelocity, double profileAcceleration, double profileDeceleration, double initialPosition){
        this.servo = servo;
        this.async = false;
        if(reversed) this.servo.setInverted(reversed);
        profile = new AsymmetricMotionProfile(profileMaxVelocity, profileAcceleration, profileDeceleration);
        isProfiled = true;
        setInitialPosition(initialPosition);
    }

    public ServoProfiled(SimpleServo servo, boolean reversed, double profileMaxVelocity, double profileAcceleration, double initialPosition){
        this(servo, reversed, profileMaxVelocity, profileAcceleration, profileAcceleration, initialPosition);
    }

    public double cachedPosition, targetPosition;

    private void setInitialPosition(double pos){
        cachedPosition = pos;
        targetPosition = pos;
        if(isProfiled) profile.setMotion(pos, pos, 0);
    }

    public void setPosition(double position){
        if(position == targetPosition) return;
        targetPosition = position;
        if(isProfiled) profile.setMotion(cachedPosition, targetPosition, profile.getSignedVelocity());
    }

    public void update(){
        if(isProfiled) profile.update();
        if(isProfiled && cachedPosition != profile.getPosition()) {
            cachedPosition = profile.getPosition();
            servo.setPosition(cachedPosition);
        }
        if(!isProfiled && cachedPosition != targetPosition) {
            cachedPosition = targetPosition;
            servo.setPosition(targetPosition);
        }
    }

    public void forceUpdate(){
        servo.setPosition(targetPosition);
    }

    public boolean isProfiled() {
        return isProfiled;
    }

    public double getTimeToMotionEnd(){
        if(!isProfiled) return 0;
        return profile.getTimeToMotionEnd();
    }
}
