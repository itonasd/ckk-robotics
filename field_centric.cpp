#include <PTBOTAtomVX.h>
#include <Bluepad32.h>
#include <unordered_map>
#include <functional>
#include <string>

#define ARM_DOWN 10
#define ARM_UP 110
#define GRIP_OPEN 135
#define GRIP_CLOSE 90
#define HANG_DOWN 90
#define HANG_MED 120
#define HANG_UP 135

typedef struct _ControllerData {
    int8_t index, l1, r1, up, down, left, right, triangle, cross, square, circle; // 0, 1 boolean
    double lx, ly, rx, ry, l2, r2; // 0.00 - 1.00 range
} ControllerData_t;

typedef struct _PIDController {
    double kp, ki, kd, integral, prev_err;
    uint32_t prev_time;
} PIDController_t;

typedef struct _TaskManager {
    std::function<void()> func;
    uint32_t triggerTime, interval;
    bool repeat, active;
} Tasks_t;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
std::unordered_map<std::string, Tasks_t> tasks;
ControllerData_t controller;
PIDController_t pid;

void taskCreate(const std::string& name, std::function<void()> func, uint32_t delay, bool repeat = false) {
    tasks[name] = {
        .func = func,
        .triggerTime = millis() + delay,
        .interval = delay,
        .repeat = repeat,
        .active = true
    };
}

void taskUpdate() {
    uint32_t now = millis();
    for (auto& [name, task] : tasks) {
        if (!task.active) continue;
        if ((now - task.triggerTime) >= 0) {
            task.func();
            if (task.repeat) task.triggerTime += task.interval;
            else task.active = false;
        }
    }
}

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (!myControllers[i]) {
            myControllers[i] = ctl;
            return;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            return;
        }
    }
}

void getControllerData(ControllerData_t* ControllerData) {
    if (!BP32.update()) return;
    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected() && ctl->hasData() && ctl->isGamepad()) {
            uint8_t buffdpad = ctl->dpad();
            uint8_t buffbutton = ctl->buttons();

            ControllerData->index = ctl->index();
            ControllerData->lx = (double) ctl->axisX() / 512.0;
            ControllerData->ly = (double) ctl->axisY() / 512.0;
            ControllerData->rx = (double) ctl->axisRX() / 512.0;
            ControllerData->ry = (double) ctl->axisRY() / 512.0;
            ControllerData->l1 = ctl->l1();
            ControllerData->r1 = ctl->r1();
            ControllerData->l2 = (double) ctl->brake() / 1020.0;
            ControllerData->r2 = (double) ctl->throttle() / 1020.0;
            ControllerData->up = (buffdpad >> 0) & 1;
            ControllerData->down = (buffdpad >> 1) & 1;
            ControllerData->left = (buffdpad >> 3) & 1;
            ControllerData->right = (buffdpad >> 2) & 1;
            ControllerData->triangle = (buffbutton >> 3) & 1;
            ControllerData->cross = (buffbutton >> 0) & 1;
            ControllerData->square = (buffbutton >> 2) & 1;
            ControllerData->circle = (buffbutton >> 1) & 1;
        }
    }
}

inline void grip(int deg) { servoWrite(1, deg); }
inline void arm(int deg)  { servoWrite(2, deg); }
inline void hang(int deg) { servoWrite(3, deg); }
double maxf64(double val1, double val2) { return (val1 > val2) ? val1 : val2; }
double minf64(double val1, double val2) { return (val1 < val2) ? val1 : val2; }
double clampf64(double val, double min, double max) { return maxf64(minf64(val, max), min); }

double wrapRads(double val) {
    while (val > PI) val -= PI*2;
    while (val < -PI) val += PI*2;
    return val;
}

bool pressOnce(const std::string& name, bool current) {
    static std::unordered_map<std::string, bool> buttonStates;
    bool clicked = current && !buttonStates[name];
    buttonStates[name] = current;
    
    return clicked;
}

double yawOffset = 0.0;
double setpoint = 0.0;
double getYaw() { return wrapRads((angleRead(YAW) - yawOffset) * PI / 180.0); }

double calculatePID(PIDController_t* pid, double setpoint, double current) {
    uint32_t now = millis();
    double dt = (now - pid->prev_time) / 1000.0;
    pid->prev_time = now;
    
    if (dt <= 0 || dt > 0.1) dt = 0.01;
    
    double error = wrapRads(setpoint - current);
    
    pid->integral += error * dt;
    pid->integral = clampf64(pid->integral, -1, 1);
    
    double derivative = (error - pid->prev_err) / dt;
    pid->prev_err = error;
    
    return clampf64(pid->kp * error + pid->ki * pid->integral + pid->kd * derivative, -1, 1);
}

double linearHeading(double yaw) {
    static bool wasTurning = false;
    double rx = controller.rx;
    bool turning = fabs(rx) > 0.1;

    if (turning) {
        wasTurning = true;
        return rx;
    }

    if (wasTurning) {
        setpoint = yaw;
        wasTurning = false;
    }

    double rotation = calculatePID(&pid, setpoint, yaw);
    if (fabs(wrapRads(setpoint - yaw)) < 0.01) rotation = 0;
    return rotation;
}

double speedControl(double value, double onControlled) {
    double absVal = fabs(value);
    if (controller.r2) return value * onControlled;
    if (absVal <= 0.1) return 0;
    if (absVal <= 0.4) return value * 0.35;
    return value;
}

void control() {
    if (pressOnce("reset", controller.square)) {
        yawOffset = angleRead(YAW);
        setpoint = 0.0;
    }

    static uint32_t serialPickState = 0;
    static bool serialRunning = false;
    if (!serialRunning && pressOnce("serialPick", controller.circle)) {
        serialRunning = true;

        switch (serialPickState) {
            case 0: {
                grip(GRIP_CLOSE);
                taskCreate("sequence1", []() {
                    arm(ARM_UP);
                }, 125);
                taskCreate("sequence2", []() {
                    grip(GRIP_OPEN);
                    hang(HANG_UP);
                }, 375);
                taskCreate("sequence3", []() {
                    arm(ARM_DOWN);
                    serialRunning = false;
                }, 400);
                break;
            }

            case 1: {
                grip(GRIP_CLOSE);
                break;
            }

            case 2: {
                grip(GRIP_OPEN);
                hang(HANG_MED);
                taskCreate("sequence4", []() {
                    arm(ARM_UP);
                }, 25);
                taskCreate("sequence5", []() {
                    grip(GRIP_CLOSE);
                    hang(HANG_DOWN);
                }, 275);
                taskCreate("sequence6", []() {
                    arm(ARM_DOWN);
                    serialRunning = false;
                }, 350);
                break;
            }

            case 3: {
                grip(GRIP_OPEN);
                break;
            }
        }

        serialPickState = (serialPickState + 1) % 4;
    }
}

void movement() {
    double yaw = getYaw();

    double lx = speedControl(controller.lx, 0.25);
    double ly = -speedControl(controller.ly, 0.15);

    double x = (cos(yaw) * lx) - (sin(yaw) * ly);
    double y = (sin(yaw) * lx) + (cos(yaw) * ly);
    double r = linearHeading(yaw);
    double d = maxf64(fabs(x) + fabs(y) + fabs(r), 1);

    double fl = (((y + x + r) / d));
    double fr = (((y - x - r) / d));
    double rl = (((y - x + r) / d));
    double rr = (((y + x - r) / d));

    motorWrite(1, fl * 50);
    motorWrite(2, fr * 50);
    motorWrite(3, rl * 50);
    motorWrite(4, rr * 50);
}

void setup() {
    Serial.begin(115200);
    initialize();
    // calibrateIMU();

    BP32.setup(&onConnectedController, &onDisconnectedController);

    pid.kp = 1;
    pid.kd = 0.1;

    arm(ARM_DOWN);
    grip(GRIP_OPEN);
    hang(HANG_DOWN);
}

void loop() {
    getControllerData(&controller);
    taskUpdate();
    control();
    movement();
}
