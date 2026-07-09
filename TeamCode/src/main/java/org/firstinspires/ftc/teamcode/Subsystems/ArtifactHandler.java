package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
// ADD INTAKE
// ADD MANUAL SHOOTING ORDER
public class ArtifactHandler {
    Spindexer spindexer;
    TransferServo transferServo;
    Hood hood;
    Intake intake;
    Flywheel flywheel;
    TurretSwivel turretSwivel;
    PinpointLocalizer pinpoint;
    ElapsedTime transferServoTimer = new ElapsedTime();
    double targetX;
    double targetY;
    double targetZ;
    double turretOffsetX;
    double turretOffsetY;
    int transferServoDelay = 1000;
    int[] motif;
    int motifIndex;
    boolean transferred = false;
    boolean verifyingTransfer = false;
    boolean verificationReady = false;
    boolean retryingTransfer = false;
    boolean transferBlocked = false;
    boolean intakeBlocked = false;
    boolean shoot = false;
    boolean enableAiming = false;
    boolean turretTargetOverride = false;
    double turretTargetOverrideAngle = 0;
    final double g = 9.81;
    Pose pose;
    private double hoodAngle = 0;
    int nextColor = -1;
    int transferVerifyDelay;
    int maxTransferRetries;
    int transferRetries = 0;
    public ArtifactHandler(Spindexer spindexer, TransferServo transferServo, Hood hood, Intake intake, Flywheel flywheel, TurretSwivel turretSwivel,PinpointLocalizer pinpoint, double targetX, double targetY, double targetZ,double turretOffsetX,double turretOffsetY,int[] motif,int transferVerifyDelay,int maxTransferRetries) {
        this.spindexer = spindexer;
        this.transferServo = transferServo;
        this.hood = hood;
        this.intake = intake;
        this.flywheel = flywheel;
        this.turretSwivel = turretSwivel;
        this.pinpoint = pinpoint;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetZ = targetZ;
        this.turretOffsetX = turretOffsetX;
        this.turretOffsetY = turretOffsetY;
        this.motif = motif;
        this.transferVerifyDelay = transferVerifyDelay;
        this.maxTransferRetries = maxTransferRetries;
    }
    public void setNextColor(int nextColor){
        this.nextColor = nextColor;
    }
    public void turnOnIntake(boolean on,boolean reverse){
        if(!on || reverse) {
            boolean wasIntakeBlocked = intakeBlocked;
            intakeBlocked = false;
            if(wasIntakeBlocked) spindexer.enable(true);
        }
        intake.enable(on && (!intakeBlocked || reverse));
        intake.setReverse(reverse);
    }
    public void setMotif(int[] motif){
        this.motif = motif;
    }
    public void setTransferServoDelay(int transferServoDelay){
        this.transferServoDelay = transferServoDelay;
    }
    public void setTargetX(double targetX){
        this.targetX = targetX;
    }
    public void setTargetY(double targetY){
        this.targetY = targetY;
    }
    public void setTargetZ(double targetZ){
        this.targetZ = targetZ;
    }
    public void requestOuttake(boolean outtake) {
        spindexer.requestOuttake(outtake);
        if(outtake) {
            spindexer.moveToColor(motif[0]);
            motifIndex = 0;
        } else {
            transferred = false;
            verifyingTransfer = false;
            verificationReady = false;
            retryingTransfer = false;
            transferRetries = 0;
            spindexer.setVerificationAlignment(false);
        }
    }
    public boolean getOuttakeState() {
        return spindexer.getOuttakeState();
    }
    public void enableAiming(boolean enableAiming){
        this.enableAiming = enableAiming;
    }
    public void setTurretTargetAngleOverride(double angle){
        turretTargetOverride = true;
        turretTargetOverrideAngle = angle;
    }
    public void clearTurretTargetAngleOverride(){
        turretTargetOverride = false;
    }
    public double getTurretAngle(){
        return turretSwivel.getCurrentAngle();
    }
    public void requestShoot(boolean shoot){
        if(!shoot) transferBlocked = false;
        this.shoot = shoot && !transferBlocked;
        if(this.shoot) clearTurretTargetAngleOverride();
        if(!spindexer.getOuttakeState() && this.shoot) requestOuttake(true);
    }
    public void update(boolean manualSort) {
        pose = pinpoint.getPose();
        if (pose == null) {
            pinpoint.update();
            pose = pinpoint.getPose();
        }
        aimTurretSwivel();
        calculateShooterParameters();
        if(intakeBlocked) spindexer.enable(false);
        spindexer.update();
        transferServo.update();
        hood.update();
        intake.update();
        flywheel.update();
        turretSwivel.update();
        if(spindexer.getOuttakeState()){
            turnOnIntake(true,false);
            flywheel.enable(true);
            transferServo.enable(true);
            turretSwivel.enable(true);
            if(spindexer.isStalled()) {
                failTransfer();
            } else if(!spindexer.slotChanger.isBusy()){
                    if(retryingTransfer) {
                        spindexer.enable(false);
                        transferServo.goUp();
                        transferred = true;
                        retryingTransfer = false;
                        transferServoTimer.reset();
                    } else if(verifyingTransfer) {
                        if(!verificationReady) {
                            verificationReady = true;
                            transferServoTimer.reset();
                        } else if(transferServoTimer.milliseconds() > transferVerifyDelay) {
                        verifyTransfer(manualSort);
                        }
                    } else if(transferred && transferServo.isGoingUp() && transferServoTimer.milliseconds() > transferServoDelay) {
                        transferServo.goDown();
                        verifyingTransfer = true;
                        verificationReady = false;
                        spindexer.enable(true);
                        spindexer.setVerificationAlignment(true);
                        transferServoTimer.reset();
                    } else if (turretSwivel.isOnTarget() && (flywheel.isOnTarget() || hood.isWithinLimits(hoodAngle)) && !transferred && shoot && !transferBlocked) {
                        if(!manualSort || spindexer.getCurrentSlotColor() != 0) {
                            spindexer.enable(false);
                            spindexer.setVerificationAlignment(false);
                            if (!transferServo.isGoingUp()) {
                                transferServoTimer.reset();
                            }
                            transferServo.goUp();
                            transferred = true;
                        }
                    }

            }
        } else if(spindexer.isStalled()) {
            failIntakeSlotChange();
        } else {
            if(enableAiming) {
                flywheel.enable(false);
                transferServo.enable(true);
                turretSwivel.enable(true);
            } else {
                flywheel.enable(false);
                transferServo.enable(false);
                turretSwivel.enable(false);
            }
        }
    }
    public void init(boolean isAuto) {
        spindexer.init(isAuto);
        flywheel.init();
        turretSwivel.init(isAuto);
    }
    private void verifyTransfer(boolean manualSort) {
        verifyingTransfer = false;
        verificationReady = false;
        if(spindexer.isCurrentSlotDetectedByVerification()) {
            if(transferRetries < maxTransferRetries) {
                transferRetries++;
                retryingTransfer = true;
                spindexer.enable(true);
                spindexer.setVerificationAlignment(false);
                transferServoTimer.reset();
            } else {
                failTransfer();
            }
            return;
        }
        finishTransfer(manualSort);
    }
    private void finishTransfer(boolean manualSort) {
        transferred = false;
        retryingTransfer = false;
        transferRetries = 0;
        spindexer.setVerificationAlignment(false);
        spindexer.clearCurrentSlot();
        spindexer.enable(true);
        if(shoot) {
            if(!manualSort) {
                motifIndex++;
                if (motifIndex == motif.length) motifIndex = 0;
                spindexer.moveToColor(motif[motifIndex]);
            } else {
                if(nextColor != -1){
                    spindexer.moveToColor(nextColor);
                    nextColor = -1;
                }
            }
        } else {
            spindexer.requestOuttake(false);
        }
        transferServoTimer.reset();
    }
    private void failTransfer() {
        transferBlocked = true;
        shoot = false;
        transferred = false;
        verifyingTransfer = false;
        verificationReady = false;
        retryingTransfer = false;
        transferRetries = 0;
        transferServo.goDown();
        spindexer.enable(true);
        spindexer.setVerificationAlignment(false);
        spindexer.requestOuttake(false);
        transferServoTimer.reset();
    }
    private void failIntakeSlotChange() {
        intakeBlocked = true;
        shoot = false;
        transferred = false;
        verifyingTransfer = false;
        verificationReady = false;
        retryingTransfer = false;
        transferRetries = 0;
        intake.enable(false);
        spindexer.enable(false);
        spindexer.setVerificationAlignment(false);
        spindexer.requestOuttake(false);
        transferServo.goDown();
        transferServoTimer.reset();
    }
    private void aimTurretSwivel() {
        if(pose == null) return;
        if(turretTargetOverride) {
            turretSwivel.setTargetAngle(turretTargetOverrideAngle);
            return;
        }
        double heading = pose.getHeading();
        turretSwivel.setTargetAngle(TurretGeometry.turretAngleToPoint(
                pose,
                turretOffsetX,
                turretOffsetY,
                targetX,
                targetY
        ));

    }
    private void calculateShooterParameters() {
        if (pose == null) return;

        double heading = pose.getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double x = pose.getX() + (turretOffsetX * cos - turretOffsetY * sin) * RobotConfig.INCHES_PER_METER;
        double y = pose.getY() + (turretOffsetX * sin + turretOffsetY * cos) * RobotConfig.INCHES_PER_METER;
        double dx = targetX - x;
        double dy = targetY - y;

        double distance = Math.hypot(dx, dy) * 0.0254;
        if (distance < 0.05) {
            hoodAngle = hood.getMinAngle() - 1;
            return;
        }
        double rps = flywheel.getRPM() / 60.0;
        double v0 = rps * 2 * Math.PI * flywheel.getRadius() * flywheel.getEfficiency();
        double middleAngle = Math.toRadians((hood.getMaxAngle() + hood.getMinAngle()) / 2.0);
        double denominator = 2 * (distance * Math.tan(middleAngle) - targetZ);

        if (denominator <= 0 || Math.abs(Math.cos(middleAngle)) < 1e-6) {
            hoodAngle = hood.getMinAngle() - 1;
            return;
        }

        double targetV0 = distance / Math.cos(middleAngle) * Math.sqrt(g / denominator);
        double targetRPM = targetV0 * 60.0 / (2 * Math.PI * flywheel.getRadius() * flywheel.getEfficiency());
        flywheel.setTargetRPM(targetRPM);
        if (v0 <= 0.1) return;

        double v2 = v0 * v0;
        double v4 = v2 * v2;
        double rootInner = v4 - (2 * g * v2 * targetZ) - (g * g * distance * distance);
        if (rootInner >= 0) {
            double tanTheta = (v2 - Math.sqrt(rootInner)) / (g * distance);
            hoodAngle = Math.toDegrees(Math.atan(tanTheta));
            hood.setAngle(hoodAngle);
        } else {
            hoodAngle = hood.getMinAngle() - 1;
        }

    }


}
//WORK IN PROGRESS
